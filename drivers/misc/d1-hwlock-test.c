// SPDX-License-Identifier: GPL-2.0
/*
 * d1-hwlock-test.c - contention test for the Allwinner sun6i hardware spinlock
 *                    against a second bus master (the D1's HiFi4 DSP).
 *
 * TEST SCAFFOLDING. Not for submission. Used to validate the sun6i hwspinlock
 * series on the Allwinner D1 by taking one lock from Linux while a FreeRTOS
 * image on the HiFi4 takes the same lock.
 *
 * The shared block lives in DDR at 0x483ff000, inside the 4 MB reserved-memory
 * carveout that also holds the firmware image (which ends near 0x480f2890):
 *   - Linux reaches it via ioremap(), uncached, and the carveout keeps the page
 *     allocator away from it;
 *   - the DSP maps 0x40000000+ as CA_BYPASS in its _cache_config(), so it is
 *     uncached on that side too.
 * Neither side caches it, so a lost update is a real locking failure and not a
 * coherency artefact.
 *
 * The DSP's own local DRAM1 at 0x00440000 was tried first and does NOT work:
 * once DSP_BOOT_RAMMAP bit 0 is cleared the SRAM belongs to the DSP and the
 * system-side alias is gone, so a C906 access to it hangs the bus forever.
 */

#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#define SHARED_PHYS_DEFAULT	0x483ff000UL
#define SHARED_SIZE		0x1000
#define SHARED_MAGIC		0x5a5a1234
#define SHARED_VERSION		1

#define TRIALS_DEFAULT		1000000

/*
 * Layout is shared verbatim with the FreeRTOS image - keep the two in sync.
 * Every field is touched with readl()/writel() so nothing is cached.
 */
struct shared {
	u32 magic;
	u32 version;
	u32 go;			/* 1 = both sides start */
	u32 stop;		/* safety stop */
	u32 target;		/* iterations the DSP should run */
	u32 counter;		/* incremented under the lock */
	u32 inside;		/* tripwire: nonzero if two masters overlap */
	u32 dsp_violations;
	u32 dsp_iters;
	u32 dsp_done;
	u32 linux_violations;
	u32 dsp_ping;		/* DSP increments this while it polls */
};

static unsigned long shared_phys = SHARED_PHYS_DEFAULT;
static unsigned int trials = TRIALS_DEFAULT;
static unsigned int lock_id;
static bool use_lock = true;
static unsigned int dsp_timeout_ms = 30000;
static bool skip_dsp;

module_param(shared_phys, ulong, 0444);
module_param(trials, uint, 0444);
module_param(lock_id, uint, 0444);
module_param(use_lock, bool, 0444);
module_param(dsp_timeout_ms, uint, 0444);
module_param(skip_dsp, bool, 0444);
MODULE_PARM_DESC(trials, "iterations on the Linux side (default 1000000)");
MODULE_PARM_DESC(lock_id, "hardware lock number to use (default 0)");
MODULE_PARM_DESC(use_lock, "1 = take the lock, 0 = negative control");
MODULE_PARM_DESC(skip_dsp, "run without a DSP partner");

static void shared_reset(struct shared __iomem *sh, u32 target)
{
	writel(0, &sh->magic);
	writel(0, &sh->go);
	writel(0, &sh->stop);
	writel(0, &sh->counter);
	writel(0, &sh->inside);
	writel(0, &sh->dsp_violations);
	writel(0, &sh->dsp_iters);
	writel(0, &sh->dsp_done);
	writel(0, &sh->linux_violations);
	writel(target, &sh->target);
	writel(SHARED_VERSION, &sh->version);
	writel(SHARED_MAGIC, &sh->magic);
}

/*
 * One burst of non-atomic read-modify-write on the shared counter.
 * Returns the number of tripwire hits, or a negative errno.
 */
static int run_burst(struct shared __iomem *sh, struct hwspinlock *lk, u32 n)
{
	unsigned int trips = 0;
	u32 i;

	for (i = 0; i < n; i++) {
		u32 cur;

		if (use_lock) {
			int ret = hwspin_lock_timeout(lk,
						      msecs_to_jiffies(1000));

			if (ret)
				return ret;
		}

		/*
		 * If the lock works, nobody else can be between the take and
		 * the free, so this must always read 0.
		 */
		if (readl(&sh->inside))
			trips++;

		writel(1, &sh->inside);

		/*
		 * Deliberately not atomic, with a gap. On uncached memory each
		 * access crosses the bus, so even this plain sequence has a
		 * wide window - which is what makes the negative control fail.
		 */
		cur = readl(&sh->counter);
		cur = readl(&sh->counter) + 1;
		writel(cur, &sh->counter);

		writel(0, &sh->inside);

		if (use_lock)
			hwspin_unlock(lk);
	}

	return trips;
}

static int __init hwlock_test_init(void)
{
	struct shared __iomem *sh;
	struct hwspinlock *lk = NULL;
	ktime_t t0, t1;
	u32 dsp_iters, counter, expected, dsp_done, dsp_trips, ping1, ping2;
	const char *verdict;
	unsigned long wait_ms = 0;
	int ret, trips;

	sh = ioremap(shared_phys, SHARED_SIZE);
	if (!sh)
		return -ENOMEM;

	pr_info("sun6i-hwlock-test: shared=%#lx trials=%u lock=%u use_lock=%d skip_dsp=%d\n",
		shared_phys, trials, lock_id, use_lock, skip_dsp);

	if (use_lock) {
		lk = hwspin_lock_request_specific(lock_id);
		if (!lk) {
			pr_err("sun6i-hwlock-test: no hardware lock %u\n", lock_id);
			ret = -ENODEV;
			goto out_unmap;
		}
	}

	/*
	 * Is the DSP executing its poll loop at all? It bumps dsp_ping every
	 * iteration there, so a changing value means "alive and waiting" -
	 * which separates "never saw magic" from "printing 'go' then died".
	 * Sampled before shared_reset() so a previous run does not confuse it.
	 */
	ping1 = readl(&sh->dsp_ping);
	msleep(100);
	ping2 = readl(&sh->dsp_ping);
	pr_info("sun6i-hwlock-test: dsp_ping %u -> %u: %s\n", ping1, ping2,
		ping1 != ping2 ? "DSP alive, polling" :
				 "DSP not polling (never started, or stuck/stopped)");

	/* Arm the shared block and release the DSP; both sides start now. */
	shared_reset(sh, trials);
	if (!skip_dsp)
		writel(1, &sh->go);

	t0 = ktime_get();
	ret = run_burst(sh, lk, trials);
	t1 = ktime_get();
	if (ret < 0) {
		pr_err("sun6i-hwlock-test: burst failed: %d\n", ret);
		goto out_put;
	}
	trips = ret;

	writel(1, &sh->stop);

	/* Wait for the DSP to finish its own burst, if there is one */
	if (!skip_dsp) {
		while (!readl(&sh->dsp_done)) {
			if (wait_ms > dsp_timeout_ms) {
				pr_err("sun6i-hwlock-test: DSP not done after %u ms\n",
				       dsp_timeout_ms);
				pr_err("sun6i-hwlock-test: dsp_ping now %u, magic %#x, go %u\n",
				       readl(&sh->dsp_ping), readl(&sh->magic),
				       readl(&sh->go));
				break;
			}
			msleep(50);
			wait_ms += 50;
		}
	}

	counter = readl(&sh->counter);
	dsp_iters = readl(&sh->dsp_iters);
	dsp_done = readl(&sh->dsp_done);
	dsp_trips = readl(&sh->dsp_violations);
	expected = trials + dsp_iters;

	/*
	 * Decide the verdict from the checks that actually matter. Note that
	 * 'counter == expected' alone is meaningless when nothing else ran:
	 * with dsp_iters == 0 it collapses to 'counter == trials', which a
	 * Linux-only run satisfies trivially.
	 */
	if (skip_dsp)
		verdict = (use_lock && counter == expected)
				  ? "SELF-TEST OK (no DSP partner)"
				  : "NON-RESULT (no DSP partner)";
	else if (!dsp_done)
		verdict = "NON-RESULT: DSP never ran";
	else if (use_lock)
		verdict = (counter == expected && !trips && !dsp_trips)
				  ? "PASS" : "FAIL";
	else
		verdict = (counter == expected && !trips && !dsp_trips)
				  ? "BROKEN TEST: negative control did not fail"
				  : "PASS (negative control failed as expected)";

	pr_info("sun6i-hwlock-test: --- use_lock=%d ---\n", use_lock);
	pr_info("sun6i-hwlock-test: linux_iters=%u dsp_iters=%u dsp_done=%u\n",
		trials, dsp_iters, dsp_done);
	pr_info("sun6i-hwlock-test: counter=%u expected=%u -> %s\n",
		counter, expected,
		counter == expected ? "EXACT" : "LOST UPDATES");
	pr_info("sun6i-hwlock-test: tripwire linux=%d dsp=%u -> %s\n",
		trips, dsp_trips,
		(trips || dsp_trips) ? "OVERLAP DETECTED" : "no overlap");
	pr_info("sun6i-hwlock-test: linux burst %lld us (%lld ns/iter)\n",
		ktime_to_us(ktime_sub(t1, t0)),
		ktime_to_ns(ktime_sub(t1, t0)) / max_t(u32, trials, 1));
	pr_info("sun6i-hwlock-test: VERDICT: %s\n", verdict);

	ret = 0;

out_put:
	if (lk)
		hwspin_lock_free(lk);
out_unmap:
	iounmap(sh);
	return ret;
}

static void __exit hwlock_test_exit(void)
{
}

module_init(hwlock_test_init);
module_exit(hwlock_test_exit);

MODULE_DESCRIPTION("Allwinner sun6i hwspinlock contention test with a HiFi4 DSP partner");
MODULE_LICENSE("GPL");
