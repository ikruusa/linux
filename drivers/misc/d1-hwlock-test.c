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
 *   - the carveout keeps the page allocator away from it;
 *   - ioremap() does NOT make it uncached on this SoC: without Svpbmt the PTE
 *     carries no attribute, and with MXSTATUS.MAEE clear the M-mode sysmap is
 *     what decides cacheability - and that map is not software-programmable.
 *     The DSP's CA_BYPASS request for 0x40000000+ does not take effect either
 *     (its own comment says so).
 *
 * So the block is cached on both sides and both sides maintain it explicitly:
 * the firmware wraps each round in xthal_dcache_region_invalidate/writeback,
 * this side in dma_sync_single_for_cpu/device() over the same 64-byte line.
 * Without that, a "lost update" is a coherency artefact rather than a locking
 * failure.
 *
 * The DSP's own local DRAM1 at 0x00440000 was tried first and does NOT work:
 * once DSP_BOOT_RAMMAP bit 0 is cleared the SRAM belongs to the DSP and the
 * system-side alias is gone, so a C906 access to it hangs the bus forever.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/hwspinlock.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define SHARED_PHYS_DEFAULT	0x483ff000UL
#define SHARED_SIZE		0x1000
#define SHARED_MAGIC		0x5a5a1234
#define SHARED_VERSION		2

#define TRIALS_DEFAULT		1000000

/*
 * Layout is shared verbatim with the FreeRTOS image - keep the two in sync.
 * The whole struct is 56 bytes at a 4 KiB-aligned base, i.e. one 64-byte cache
 * line, which is exactly the granularity the maintenance below works on.
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
	u32 dsp_bench_req;	/* 1 = DSP runs its own lock benchmark */
	u32 dsp_bench_done;	/* DSP sets this when that benchmark finished */
};

static unsigned long shared_phys = SHARED_PHYS_DEFAULT;
static unsigned int trials = TRIALS_DEFAULT;
static unsigned int lock_id;
static bool use_lock = true;
static unsigned int dsp_timeout_ms = 30000;
static bool skip_dsp;
static unsigned int bench;

module_param(shared_phys, ulong, 0444);
module_param(trials, uint, 0444);
module_param(lock_id, uint, 0444);
module_param(use_lock, bool, 0444);
module_param(dsp_timeout_ms, uint, 0444);
module_param(skip_dsp, bool, 0444);
module_param(bench, uint, 0444);
MODULE_PARM_DESC(trials, "iterations on the Linux side (default 1000000)");
MODULE_PARM_DESC(lock_id, "hardware lock number to use (default 0)");
MODULE_PARM_DESC(use_lock, "1 = take the lock, 0 = negative control");
MODULE_PARM_DESC(skip_dsp, "run without a DSP partner");
MODULE_PARM_DESC(bench, "0 = contention test, 1 = C906 take/free cost, 2 = 1 + DSP take/free cost, 3 = 2 + bare register cost, 4 = 3 + framework split");

/*
 * Cross-core cache maintenance, the same thing the DSP firmware does with
 * xthal_dcache_region_invalidate()/writeback(). The DMA API is the way to get
 * at the C906's CMO instructions from a driver: dma-direct sees a
 * non-coherent device (the soc node carries dma-noncoherent) and runs the
 * T-Head th.dcache.cpa/civa ops. sh_dev is the hwspinlock platform device.
 */
#define SHARED_SYNC_BYTES	64

static struct platform_device *sh_pdev;
static struct device *sh_dev;

/* Pull in what the peer wrote, before reading the block. */
static inline void sh_sync_in(void)
{
	if (sh_dev)
		dma_sync_single_for_cpu(sh_dev, shared_phys, SHARED_SYNC_BYTES,
					DMA_FROM_DEVICE);
	mb();
}

/* Make our writes visible, before releasing the lock. */
static inline void sh_sync_out(void)
{
	mb();
	if (sh_dev)
		dma_sync_single_for_device(sh_dev, shared_phys, SHARED_SYNC_BYTES,
					   DMA_TO_DEVICE);
}

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
	writel(0, &sh->dsp_bench_req);
	writel(0, &sh->dsp_bench_done);
	writel(target, &sh->target);
	writel(SHARED_VERSION, &sh->version);
	writel(SHARED_MAGIC, &sh->magic);
	sh_sync_out();
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
		 * The lock only guarantees exclusion. The shared line is cached on
		 * both sides, so each master has to fetch the other's writes itself:
		 * invalidate on entry, clean on the way out - and clean *before*
		 * releasing the lock, or the peer could take it while our update is
		 * still only in our cache.
		 */
		sh_sync_in();

		/*
		 * If the lock works, nobody else can be between the take and
		 * the free, so this must always read 0.
		 */
		if (readl(&sh->inside))
			trips++;

		writel(1, &sh->inside);

		/*
		 * Deliberately not atomic, with a gap: the read, the re-read and
		 * the store are separate bus accesses, so even this plain sequence
		 * has a wide window - which is what makes the negative control
		 * fail.
		 */
		cur = readl(&sh->counter);
		cur = readl(&sh->counter) + 1;
		writel(cur, &sh->counter);

		writel(0, &sh->inside);

		sh_sync_out();

		if (use_lock)
			hwspin_unlock(lk);
	}

	return trips;
}

/*
 * Benchmarks. These run with go == 0, so the DSP never touches the lock and
 * what is measured is the primitive itself rather than waiting for the peer.
 * The baseline uses a word in the same page that the DSP never touches, so it
 * is not perturbed by the polling writeback of the shared cache line.
 */
#define BENCH_WORD_OFFSET	0x800

static void __iomem *bench_word(struct shared __iomem *sh)
{
	return (void __iomem *)((char __iomem *)sh + BENCH_WORD_OFFSET);
}

static s64 bench_read(struct shared __iomem *sh, u32 n)
{
	void __iomem *w = bench_word(sh);
	ktime_t t0 = ktime_get();
	u32 i, sink = 0;

	for (i = 0; i < n; i++)
		sink += readl(w);

	/* keep the loop from being optimised away */
	if (sink == 0xdeadbeef)
		writel(sink, w);

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

static s64 bench_write(struct shared __iomem *sh, u32 n)
{
	void __iomem *w = bench_word(sh);
	ktime_t t0 = ktime_get();
	u32 i;

	for (i = 0; i < n; i++)
		writel(0, w);

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

static s64 bench_take_free(struct hwspinlock *lk, u32 n)
{
	ktime_t t0 = ktime_get();
	u32 i;

	for (i = 0; i < n; i++) {
		int ret = hwspin_lock_timeout(lk, msecs_to_jiffies(1000));

		if (ret)
			return ret;
		hwspin_unlock(lk);
	}

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

/*
 * Take/free straight on the register: one readl() to take (the block returns 0
 * on success), one writel(0) to release, as sun6i_hwspinlock_trylock()/unlock()
 * do - but without the spinlock, the mb()s or the timeout bookkeeping. Splits
 * the measured cost into "the register is slow" and "the framework is slow".
 */
static s64 bench_raw_take_free(void __iomem *lock_reg, u32 n)
{
	ktime_t t0 = ktime_get();
	u32 i, missed = 0;

	for (i = 0; i < n; i++) {
		if (readl(lock_reg)) {
			missed++;
			continue;
		}
		writel(0, lock_reg);
	}

	if (missed)
		pr_warn("sun6i-hwlock-test: raw bench: %u takes failed\n", missed);

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

/* Same, through the API's raw mode: adds the two mb() calls, nothing else. */
static s64 bench_trylock_raw(struct hwspinlock *lk, u32 n)
{
	ktime_t t0 = ktime_get();
	u32 i, missed = 0;

	for (i = 0; i < n; i++) {
		if (hwspin_trylock_raw(lk)) {
			missed++;
			continue;
		}
		hwspin_unlock_raw(lk);
	}

	if (missed)
		pr_warn("sun6i-hwlock-test: trylock_raw bench: %u takes failed\n",
			missed);

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

/* Same register loop, plus the two mb() the hwspinlock core issues. */
static s64 bench_raw_take_free_mb(void __iomem *lock_reg, u32 n)
{
	ktime_t t0 = ktime_get();
	u32 i;

	for (i = 0; i < n; i++) {
		if (readl(lock_reg))
			continue;
		mb();
		writel(0, lock_reg);
		mb();
	}

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

/*
 * CPU clock probe: eight dependent adds, one cycle apart on an in-order core,
 * so ns/iteration * clock = 8. Turns every other number here into cycles and
 * shows whether a cost is the code or just a slow core.
 */
#define DEP_ADDI(x, k) asm volatile("addi %0, %0, " #k : "+r"(x))

static s64 bench_cpu_speed(u32 n)
{
	ktime_t t0 = ktime_get();
	u64 x = 0;
	u32 i;

	for (i = 0; i < n; i++) {
		DEP_ADDI(x, 1);
		DEP_ADDI(x, 2);
		DEP_ADDI(x, 3);
		DEP_ADDI(x, 4);
		DEP_ADDI(x, 5);
		DEP_ADDI(x, 6);
		DEP_ADDI(x, 7);
		DEP_ADDI(x, 8);
	}

	if (x == 42)
		pr_info("sun6i-hwlock-test: unreachable %llu\n", x);

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

static s64 bench_atomic_api(struct hwspinlock *lk, u32 n)
{
	ktime_t t0 = ktime_get();
	u32 i;

	for (i = 0; i < n; i++) {
		if (hwspin_lock_timeout_in_atomic(lk, 1000))
			break;
		hwspin_unlock_in_atomic(lk);
	}

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

static s64 bench_spinlock_pair(u32 n)
{
	static DEFINE_SPINLOCK(dummy);
	ktime_t t0 = ktime_get();
	unsigned long flags;
	u32 i;

	for (i = 0; i < n; i++) {
		spin_lock_irqsave(&dummy, flags);
		spin_unlock_irqrestore(&dummy, flags);
	}

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

static noinline void bench_noop(void)
{
}

/* One call and return, nothing else - the unit cost of the call machinery. */
static s64 bench_call_overhead(u32 n)
{
	ktime_t t0 = ktime_get();
	u32 i;

	for (i = 0; i < n; i++)
		bench_noop();

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

/* One load from a 256-byte buffer that always stays hot in L1. */
static s64 bench_cached_read(u32 n)
{
	static u32 buf[64] __cacheline_aligned;
	ktime_t t0 = ktime_get();
	u32 i, sink = 0;

	buf[0] = 1;

	for (i = 0; i < n; i++)
		sink += READ_ONCE(buf[i & 63]);

	if (sink == 0xdeadbeef)
		pr_info("sun6i-hwlock-test: unreachable %u\n", sink);

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

static s64 bench_timeout_math(u32 n){
	ktime_t t0 = ktime_get();
	unsigned long acc = 0;
	u32 i;

	for (i = 0; i < n; i++)
		acc += msecs_to_jiffies(1000) + jiffies;

	if (acc == ULONG_MAX)
		pr_info("sun6i-hwlock-test: %lu\n", acc);

	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

/* Lock N of the sun6i bank lives at SPINLOCK_LOCK_REGN + 4 * N (0x100). */
#define HWLOCK_LOCK_REGN	0x100

static int run_bench(struct shared __iomem *sh, struct hwspinlock *lk, u32 n)
{
	unsigned long wait_ms = 0;
	ktime_t t0;
	s64 rd_ns, wr_ns, lk_ns, raw_ns = 0, rawapi_ns = 0;
	s64 atomic_ns = 0, spin_ns = 0, time_ns = 0, mb_ns = 0, cpu_ns = 0;
	s64 call_ns = 0, cread_ns = 0;
	void __iomem *lock_base = NULL, *lock_reg = NULL;

	/* magic set, go stays 0: the DSP polls but never takes the lock */
	shared_reset(sh, n);

	pr_info("sun6i-hwlock-test: bench %u, %u iterations, lock %u, DSP idle\n",
		bench, n, lock_id);

	/*
	 * bench 3 repeats the take/free on the bare register, to separate the
	 * cost of the bus from the cost of the framework around it.
	 */
	if (bench >= 3) {
		struct device_node *np;

		np = of_find_compatible_node(NULL, NULL,
					     "allwinner,sun20i-d1-hwspinlock");
		if (np) {
			lock_base = of_iomap(np, 0);
			of_node_put(np);
		}
		if (lock_base)
			lock_reg = lock_base + HWLOCK_LOCK_REGN + 4 * lock_id;
		else
			pr_warn("sun6i-hwlock-test: no hwlock node, raw bench skipped\n");
	}

	rd_ns = bench_read(sh, n);
	wr_ns = bench_write(sh, n);
	if (lock_reg) {
		raw_ns = bench_raw_take_free(lock_reg, n);
		rawapi_ns = bench_trylock_raw(lk, n);
	}
	if (bench >= 4) {
		atomic_ns = bench_atomic_api(lk, n);
		spin_ns = bench_spinlock_pair(n);
		time_ns = bench_timeout_math(n);
		cpu_ns = bench_cpu_speed(n);
		call_ns = bench_call_overhead(n);
		cread_ns = bench_cached_read(n);
		if (lock_reg)
			mb_ns = bench_raw_take_free_mb(lock_reg, n);
	}
	lk_ns = bench_take_free(lk, n);
	if (lk_ns < 0) {
		pr_err("sun6i-hwlock-test: take/free failed: %lld\n", lk_ns);
		if (lock_base)
			iounmap(lock_base);
		return (int)lk_ns;
	}

	pr_info("sun6i-hwlock-test:   uncached read  %8lld ns/iter\n",
		rd_ns / max_t(u32, n, 1));
	pr_info("sun6i-hwlock-test:   uncached write %8lld ns/iter\n",
		wr_ns / max_t(u32, n, 1));
	if (lock_reg) {
		pr_info("sun6i-hwlock-test:   lock reg r+w   %8lld ns/iter\n",
			raw_ns / max_t(u32, n, 1));
		pr_info("sun6i-hwlock-test:   trylock_raw    %8lld ns/iter\n",
			rawapi_ns / max_t(u32, n, 1));
	}
	if (bench >= 4) {
		if (lock_reg)
			pr_info("sun6i-hwlock-test:   reg + 2 mb     %8lld ns/iter\n",
				mb_ns / max_t(u32, n, 1));
		pr_info("sun6i-hwlock-test:   atomic api     %8lld ns/iter\n",
			atomic_ns / max_t(u32, n, 1));
		pr_info("sun6i-hwlock-test:   spinlock pair  %8lld ns/iter\n",
			spin_ns / max_t(u32, n, 1));
		pr_info("sun6i-hwlock-test:   jiffies+msecs  %8lld ns/iter\n",
			time_ns / max_t(u32, n, 1));
		pr_info("sun6i-hwlock-test:   cpu, 8 adds    %8lld ns/iter (~%lld MHz)\n",
			cpu_ns / max_t(u32, n, 1),
			8000 / max_t(s64, cpu_ns / max_t(u32, n, 1), 1));
		pr_info("sun6i-hwlock-test:   noop call      %8lld ns/iter\n",
			call_ns / max_t(u32, n, 1));
		pr_info("sun6i-hwlock-test:   cached read    %8lld ns/iter\n",
			cread_ns / max_t(u32, n, 1));
	}
	pr_info("sun6i-hwlock-test:   take + free    %8lld ns/iter\n",
		lk_ns / max_t(u32, n, 1));

	if (lock_base)
		iounmap(lock_base);

	if (bench < 2 || skip_dsp)
		return 0;

	/*
	 * Ask the DSP for its own take/free cost. It is timed from here in wall
	 * clock, because its cycle counter runs at a tick rate we cannot map to
	 * nanoseconds. One poll delay (~10 ms) is included, which is noise for
	 * the iteration counts used here.
	 */
	writel(0, &sh->dsp_bench_done);
	writel(1, &sh->dsp_bench_req);

	t0 = ktime_get();
	while (!readl(&sh->dsp_bench_done)) {
		if (wait_ms > dsp_timeout_ms) {
			pr_err("sun6i-hwlock-test: DSP bench not done after %u ms\n",
			       dsp_timeout_ms);
			pr_err("sun6i-hwlock-test: dsp_ping %u - firmware older than FW_REV 5?\n",
			       readl(&sh->dsp_ping));
			return -ETIMEDOUT;
		}
		msleep(20);
		wait_ms += 20;
	}

	pr_info("sun6i-hwlock-test:   DSP take+free  %8lld ns/iter (wall clock)\n",
		ktime_to_ns(ktime_sub(ktime_get(), t0)) / max_t(u32, n, 1));

	return 0;
}

static int __init hwlock_test_init(void)
{
	struct shared __iomem *sh;
	struct hwspinlock *lk = NULL;
	struct device_node *np;
	ktime_t t0, t1;
	u32 dsp_iters, counter, expected, dsp_done, dsp_trips, ping1, ping2;
	const char *verdict;
	unsigned long wait_ms = 0;
	int ret, trips;

	sh = ioremap(shared_phys, SHARED_SIZE);
	if (!sh)
		return -ENOMEM;

	/*
	 * A device from the soc node, only to reach the CMO instructions through
	 * the DMA API. The node exists whether or not the hwspinlock driver is
	 * bound, so this works for the use_lock=0 control as well.
	 */
	np = of_find_compatible_node(NULL, NULL,
				     "allwinner,sun20i-d1-hwspinlock");
	if (np) {
		sh_pdev = of_find_device_by_node(np);
		of_node_put(np);
	}
	if (sh_pdev) {
		sh_dev = &sh_pdev->dev;
	} else {
		pr_warn("sun6i-hwlock-test: no hwlock device, cannot maintain the shared line - results mean nothing with the D-cache on\n");
	}

	pr_info("sun6i-hwlock-test: shared=%#lx trials=%u lock=%u use_lock=%d skip_dsp=%d\n",
		shared_phys, trials, lock_id, use_lock, skip_dsp);

	if (use_lock || bench) {
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
	sh_sync_in();
	ping2 = readl(&sh->dsp_ping);
	pr_info("sun6i-hwlock-test: dsp_ping %u -> %u: %s\n", ping1, ping2,
		ping1 != ping2 ? "DSP alive, polling" :
				 "DSP not polling (never started, or stuck/stopped)");

	if (bench) {
		ret = run_bench(sh, lk, trials);
		goto out_put;
	}

	/* Arm the shared block and release the DSP; both sides start now. */
	shared_reset(sh, trials);
	if (!skip_dsp) {
		writel(1, &sh->go);
		sh_sync_out();
	}

	t0 = ktime_get();
	ret = run_burst(sh, lk, trials);
	t1 = ktime_get();
	if (ret < 0) {
		pr_err("sun6i-hwlock-test: burst failed: %d\n", ret);
		goto out_put;
	}
	trips = ret;

	writel(1, &sh->stop);
	sh_sync_out();

	/* Wait for the DSP to finish its own burst, if there is one */
	if (!skip_dsp) {
		for (;;) {
			sh_sync_in();
			if (readl(&sh->dsp_done))
				break;
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

	sh_sync_in();
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
	if (sh_pdev)
		put_device(&sh_pdev->dev);
	sh_dev = NULL;
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
