/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/freezer.h>
#include <linux/hugetlb_inline.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/time.h>

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
#include <mt-plat/aee.h>
#include <disp_assert_layer.h>
static uint32_t in_lowmem;
#endif

#ifdef CONFIG_HIGHMEM
#include <linux/highmem.h>
#endif

#ifdef CONFIG_MTK_ION
#include "mtk/ion_drv.h"
#endif

#ifdef CONFIG_MTK_GPU_SUPPORT
#include <mt-plat/mtk_gpu_utility.h>
#endif

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
#define CONVERT_ADJ(x) ((x * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE)
#define REVERT_ADJ(x)  (x * (-OOM_DISABLE + 1) / OOM_SCORE_ADJ_MAX)
#else
#define CONVERT_ADJ(x) (x)
#define REVERT_ADJ(x) (x)
#endif /* CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES */

static short lowmem_debug_adj = CONVERT_ADJ(0);
#ifdef CONFIG_MT_ENG_BUILD
#ifdef CONFIG_MTK_AEE_FEATURE
static short lowmem_kernel_warn_adj = CONVERT_ADJ(0);
#endif
#define output_expect(x) likely(x)
static uint32_t enable_candidate_log = 1;
#define LMK_LOG_BUF_SIZE 500
static uint8_t lmk_log_buf[LMK_LOG_BUF_SIZE];
#else
#define output_expect(x) unlikely(x)
static uint32_t enable_candidate_log;
#endif
static DEFINE_SPINLOCK(lowmem_shrink_lock);

#define CREATE_TRACE_POINTS
#include "trace/lowmemorykiller.h"

static uint32_t lowmem_debug_level = 1;
static short lowmem_adj[9] = {
	0,
	1,
	6,
	12,
};

u8 empty_slot = 0xff;           /* bit[n] = 0, means there is no task
                                 * which adj is great or equal to the binded
                                 * lowmem_adj[x] */
unsigned long jiffies_empty_slot = INITIAL_JIFFIES;

static int lowmem_adj_size = 9;
int lowmem_minfree[9] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 9;

#ifdef CONFIG_HIGHMEM
static int total_low_ratio = 1;
#endif

static struct task_struct *lowmem_deathpending;
static unsigned long lowmem_deathpending_timeout;
static unsigned long dump_meminfo_timeout;
static struct workqueue_struct *dumpmem_workqueue;
static void dump_meminfo(struct work_struct *work);
static DECLARE_WORK(dumpmem_work, dump_meminfo);

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

DEFINE_MUTEX(profile_mutex);

struct profile
{
    unsigned long timeout;
    unsigned long nr_to_scan;   /* summation of sc->nr_to_scan in time period */
    int fire;                   /* numbers of call for LMK shrinker */
    int select;                 /* numbers of killing */
    int reason_adj_max;         /* escape reason: min_score_adj == OOM_SCORE_ADJ_MAX + 1 */
    int reason_tif_memdie;      /* escape reason: selecting a teriminating task */
    int reason_nop;             /* escape reason: no operation + selected */
    int ofree;
    int ofile;
    unsigned long pressure;     /* pressure for previous profile */
};

static struct profile g_lmk_profile = {
        .timeout = INITIAL_JIFFIES,
};

#define LMK_PROFILE_INTERVAL (0.25)
//Lower bound for cache = 56MB
#define LMK_LOW_BOUND_CACHE 14336
#define LMK_LOW_BOUND_NR 20
#define LMK_LOW_BOUND_FIRE ((unsigned long)(4000*LMK_PROFILE_INTERVAL))

/* standardize memory pressure: 0~10 */
#define standardize_lmk_pressure(p)                         \
    do {                                                    \
        p = (p/(LMK_LOW_BOUND_NR*LMK_LOW_BOUND_FIRE/10));   \
        if (p > 10)                                         \
            p = 10;                                         \
    } while(0)

#define profile_reset(p)                               \
    do {                                               \
        memset(p, 0, sizeof(struct profile));          \
        p->timeout = jiffies + (unsigned long)(LMK_PROFILE_INTERVAL*HZ); \
    } while(0)

static int profile_show(struct profile *p, u8 task_bitmap_mask)
{
    int retv = 0;
    int pressure;
    unsigned long sp;

    if (p->timeout == INITIAL_JIFFIES) {
        lowmem_print(2, "totalreserve_pages=%lu, LMK_LOW_BOUND_FIRE=%lu\n", totalreserve_pages, LMK_LOW_BOUND_FIRE);
        profile_reset(p);
        return 0;
    }

    task_bitmap_mask >>= 1;     /* excludes the most important processes. */
    mutex_lock(&profile_mutex);
    if (p->fire > 0 && time_after(jiffies, p->timeout)) {
        pressure = p->nr_to_scan;
        lowmem_print(2, "efficicy: kill=%d, shrink=%d, adj_max=%d, memdie=%d, nop=%d, ofree=%d, ofile=%d, pressure=%d\n",
                     p->select, p->fire, p->reason_adj_max, p->reason_tif_memdie,
                     p->reason_nop - p->select,
                     p->ofree/p->fire, p->ofile/p->fire, pressure);

        if (p->select == 0) {
                /* No killed process in previous profiling */
                sp = pressure;
                standardize_lmk_pressure(sp);
                switch (sp) {
                case 10:
                        /* Highest pressure score.
                         * Basically, we want to selection some one to be killed. */
                        retv = 1;
                        if ((empty_slot & task_bitmap_mask) == 0 &&
                            (p->ofile/p->fire > LMK_LOW_BOUND_CACHE))
                                /* All other than foreground app has been killed
                                 * Force selection when cache pages is low enough.
                                 * This prevent killing the FG app soon, but it may
                                 * cause system lags because low memory. */
                                retv = 0;
                        break;
                case 9:
                case 8:
                case 7:
                case 6:
                        /* Higher pressure score.
                         * Kill some one that is not a FG process */
                        if (empty_slot & task_bitmap_mask)
                                retv = 1;
                        break;
                default:;
                }
                if (retv)
                        lowmem_print(1, "force selecting(p:%lu, ofile: %u)\n", sp, p->ofile/p->fire);
        }
        profile_reset(p);
        p->pressure = pressure;
    }
    mutex_unlock(&profile_mutex);
    return retv;
}

#define PSS_SHIFT 12
struct mem_size_stats {
    struct vm_area_struct *vma;
    unsigned long resident;
    unsigned long shared_clean;
    unsigned long shared_dirty;
    unsigned long private_clean;
    unsigned long private_dirty;
    unsigned long referenced;
    unsigned long anonymous;
    unsigned long anonymous_thp;
    unsigned long swap;
    unsigned long nonlinear;
    u64 pss;
};

static void dump_meminfo(struct work_struct *work)
{
    struct task_struct *tsk;
    int tasksize;

    lowmem_print(1,"This is dump by LMK while adj = 0\n");

    for_each_process(tsk){
        struct task_struct *p;
        int oom_score_adj;

        if (tsk->flags & PF_KTHREAD)
            continue;

        p = find_lock_task_mm(tsk);
        if (!p)
            continue;

        oom_score_adj = p->signal->oom_score_adj;

        tasksize = get_mm_rss(p->mm);

        task_unlock(p);
        if (tasksize <= 0)
            continue;
        lowmem_print(1, "PID:%d P_Name:%s Adj:%d Size:%d KB\n"
          ,p->pid, p->comm, oom_score_adj, tasksize *4);
    }
}

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data);

static struct notifier_block task_nb = {
	.notifier_call	= task_notify_func,
};

static int
task_notify_func(struct notifier_block *self, unsigned long val, void *data)
{
	struct task_struct *task = data;

	if (task == lowmem_deathpending)
		lowmem_deathpending = NULL;

	return NOTIFY_DONE;
}

static unsigned long lowmem_count(struct shrinker *s,
				  struct shrink_control *sc)
{
#ifdef CONFIG_FREEZER
	/* Do not allow LMK to work when system is freezing */
	if (pm_freezing)
		return 0;
#endif
	return global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
}

static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	struct task_struct *selected_qos = NULL;
	unsigned long rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
        int selected_tasksize = 0, selected_qos_tasksize = 0;
        short selected_oom_score_adj, selected_qos_oom_score_adj;
        bool selected_mark_as_prefer_kill = 0;
	int array_size = ARRAY_SIZE(lowmem_adj);
        int force_select = 0;
        u8 task_bitmap = 0;          /* bitmap indicates the associated adj category is empty or not */
        u8 task_bitmap_mask = 0;
        u8 candidate_slot = 0;
	int other_free = global_page_state(NR_FREE_PAGES) - totalreserve_pages;
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM) -
						total_swapcache_pages();

	int print_extra_info = 0;
	static unsigned long lowmem_print_extra_info_timeout;

#ifdef CONFIG_MTK_GMO_RAM_OPTIMIZE
	int other_anon = global_page_state(NR_INACTIVE_ANON) - global_page_state(NR_ACTIVE_ANON);
#endif
#ifdef CONFIG_MT_ENG_BUILD
	/* dump memory info when framework low memory*/
	int pid_dump = -1; /* process which need to be dump */
	/* int pid_sec_mem = -1; */
	int max_mem = 0;
	static int pid_flm_warn = -1;
	static unsigned long flm_warn_timeout;
	int log_offset = 0, log_ret;
#endif /* CONFIG_MT_ENG_BUILD*/
	/*
	* If we already have a death outstanding, then
	* bail out right away; indicating to vmscan
	* that we have nothing further to offer on
	* this pass.
	*
	*/
	if (lowmem_deathpending &&
	    time_before_eq(jiffies, lowmem_deathpending_timeout))
		return SHRINK_STOP;

	/* Subtract CMA free pages from other_free if this is an unmovable page allocation */
	if (IS_ENABLED(CONFIG_CMA))
		if (!(sc->gfp_mask & __GFP_MOVABLE))
			other_free -= global_page_state(NR_FREE_CMA_PAGES);

	if (!spin_trylock(&lowmem_shrink_lock)) {
		lowmem_print(4, "lowmem_shrink lock failed\n");
		return SHRINK_STOP;
	}

	/* Let other_free be positive or zero */
	if (other_free < 0) {
		/* lowmem_print(1, "Original other_free [%d] is too low!\n", other_free); */
		other_free = 0;
	}

#if defined(CONFIG_64BIT) && defined(CONFIG_SWAP)
	/* Halve other_free if there is less free swap */
	if (vm_swap_full()) {
		lowmem_print(3, "Halve other_free %d\n", other_free);
		other_free >>= 1;
	}
#endif

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
			min_score_adj = lowmem_adj[i];
			break;
		}
	}

        switch (array_size) {
            case 6:
                task_bitmap_mask = 0x3f;
                break;
            case 4:
                task_bitmap_mask = 0x0f;
                break;
            default:
                pr_info("array size of lowmem_minfree_size should be 4 or 6\n");
                WARN_ON(1);
        }

#ifdef CONFIG_MTK_GMO_RAM_OPTIMIZE /* Need removal */
	if (min_score_adj < 9 && other_anon > 70 * 256) {
		/* if other_anon > 70MB, don't kill adj <= 8 */
		min_score_adj = 9;
	}
#endif

        force_select = profile_show(&g_lmk_profile, task_bitmap_mask);
        if (force_select > 0) {
                min_score_adj = lowmem_adj[0];
        }
        else {
                candidate_slot = task_bitmap_mask;
                for (i = 0; i < array_size; i++) {
                        minfree = lowmem_minfree[i];
                        if (other_free < minfree && other_file < minfree) {
                                min_score_adj = lowmem_adj[i];
                                break;
                        }
                candidate_slot >>= 1;
                }

                if (min_score_adj < OOM_SCORE_ADJ_MAX + 1) {
                        mutex_lock(&profile_mutex);
                        /* want to select something
                        * We check whether the empty timestamp is out of date. */
                        if ((empty_slot & candidate_slot) == 0 &&
                                jiffies_empty_slot != INITIAL_JIFFIES &&
                                time_before_eq(jiffies, jiffies_empty_slot)) {
                                lowmem_print(3, "skip selection (adj: %d, time remains: %u ms)\n",
                                        min_score_adj, jiffies_to_msecs(jiffies_empty_slot - jiffies));
                                min_score_adj = OOM_SCORE_ADJ_MAX + 1;
                        }
                        mutex_unlock(&profile_mutex);
                }
        }
        g_lmk_profile.fire++;
        g_lmk_profile.ofree += other_free;
        g_lmk_profile.ofile += other_file;

         if (sc->nr_to_scan > 0) {
                g_lmk_profile.nr_to_scan += sc->nr_to_scan;
        }

	lowmem_print(3, "lowmem_scan %lu, %x, ofree %d %d, ma %hd\n",
			sc->nr_to_scan, sc->gfp_mask, other_free,
			other_file, min_score_adj);

	if (!force_select && min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_scan %lu, %x, return 0\n",
			     sc->nr_to_scan, sc->gfp_mask);

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
		/*
		* disable indication if low memory
		*/
		if (in_lowmem) {
			in_lowmem = 0;
			/* DAL_LowMemoryOff(); */
			lowmem_print(1, "LowMemoryOff\n");
		}
#endif
                g_lmk_profile.reason_adj_max++;
		spin_unlock(&lowmem_shrink_lock);
		return 0;
	}

	selected_oom_score_adj = min_score_adj;
        selected_qos_oom_score_adj = min_score_adj;

	/* add debug log */
	if (output_expect(enable_candidate_log)) {
		if (min_score_adj <= lowmem_debug_adj) {
			if (time_after_eq(jiffies, lowmem_print_extra_info_timeout)) {
				lowmem_print_extra_info_timeout = jiffies + HZ;
				print_extra_info = 1;
			}
		}

		if (print_extra_info) {
			lowmem_print(1, "Free memory other_free: %d, other_file:%d pages\n", other_free, other_file);
#ifdef CONFIG_MT_ENG_BUILD
			log_offset = snprintf(lmk_log_buf, LMK_LOG_BUF_SIZE, "%s",
#else
			lowmem_print(1,
#endif
#ifdef CONFIG_ZRAM
					"<lmk>  pid  adj  score_adj     rss   rswap name\n");
#else
					"<lmk>  pid  adj  score_adj     rss name\n");
#endif
		}
	}

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;
		short oom_qos;
		unsigned long task_life_time;
		struct timespec delta;
		short oom_qos_endurance;
		bool oom_qos_prefer_kill;

		if (tsk->flags & PF_KTHREAD)
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		if (test_tsk_thread_flag(p, TIF_MEMDIE) &&
		    time_before_eq(jiffies, lowmem_deathpending_timeout)) {
#ifdef CONFIG_MT_ENG_BUILD
			static pid_t last_dying_pid;

			if (last_dying_pid != p->pid) {
				lowmem_print(1, "lowmem_shrink return directly, due to  %d (%s) is dying\n",
					p->pid, p->comm);
				last_dying_pid = p->pid;
			}
#endif
			task_unlock(p);
			rcu_read_unlock();
                        /* if pressure is high enough,
                         * select the other one even if someone is teriminting. */
                        if (force_select > 0) continue;
                        g_lmk_profile.reason_tif_memdie++;
			spin_unlock(&lowmem_shrink_lock);
			return SHRINK_STOP;
		}
		oom_score_adj = p->signal->oom_score_adj;
                oom_qos = p->signal->oom_qos;
                get_oom_qos_endurance(oom_qos_endurance, oom_qos);
                oom_qos_prefer_kill = is_prefer_to_kill(oom_qos);
                /* mark the associated category is empty or not
                 * The highest value of task_bitmap is 'task_bitmap_mask',
                 * means all slots are not empty. */
                for (i = array_size - 1; i >= 0 && (task_bitmap & task_bitmap_mask) < task_bitmap_mask; i--) {
                        if (oom_score_adj >= lowmem_adj[i]) {
                                task_bitmap |= 1 << (array_size - 1 - i);
                                break;
                        }
                }

		if (output_expect(enable_candidate_log)) {
			if (print_extra_info) {
#ifdef CONFIG_MT_ENG_BUILD
log_again:
				log_ret = snprintf(lmk_log_buf+log_offset, LMK_LOG_BUF_SIZE-log_offset,
#else
				lowmem_print(1,
#endif
#ifdef CONFIG_ZRAM
						"<lmk>%5d%5d%11d%8lu%8lu %s\n", p->pid,
						REVERT_ADJ(oom_score_adj), oom_score_adj,
						get_mm_rss(p->mm),
						get_mm_counter(p->mm, MM_SWAPENTS), p->comm);
#else /* CONFIG_ZRAM */
						"<lmk>%5d%5d%11d%8lu %s\n", p->pid,
						REVERT_ADJ(oom_score_adj), oom_score_adj,
						get_mm_rss(p->mm), p->comm);
#endif

#ifdef CONFIG_MT_ENG_BUILD
				if ((log_offset + log_ret) >= LMK_LOG_BUF_SIZE || log_ret < 0) {
					*(lmk_log_buf + log_offset) = '\0';
					lowmem_print(1, "\n%s", lmk_log_buf);
					/* pr_err("lmk log overflow log_offset:%d\n", log_offset); */
					log_offset = 0;
					memset(lmk_log_buf, 0x0, LMK_LOG_BUF_SIZE);
					goto log_again;
				} else
					log_offset += log_ret;
#endif
			}
		}

#ifdef CONFIG_MT_ENG_BUILD
		tasksize = get_mm_rss(p->mm);
#ifdef CONFIG_ZRAM
		tasksize += get_mm_counter(p->mm, MM_SWAPENTS);
#endif
		/*
		* dump memory info when framework low memory:
		* record the first two pid which consumed most memory.
		*/
		if (tasksize > max_mem) {
			max_mem = tasksize;
			/* pid_sec_mem = pid_dump; */
			pid_dump = p->pid;
		}

		if (p->pid == pid_flm_warn &&
			time_before_eq(jiffies, flm_warn_timeout)) {
			task_unlock(p);
			continue;
		}
#endif

		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}

#ifndef CONFIG_MT_ENG_BUILD
		tasksize = get_mm_rss(p->mm);
#ifdef CONFIG_ZRAM
		tasksize += get_mm_counter(p->mm, MM_SWAPENTS);
#endif
#endif
		task_unlock(p);
		if (tasksize <= 0)
			continue;
                //ASUS_BSP:ignore media if it's working
                if(oom_score_adj == 117 && !strcmp(p->comm,"d.process.media")){
                    lowmem_print(2, "adj == 117 and media, not killed !!!!\n");
                    continue;
                }

                if(!strcmp(p->comm,"FinalizerDaemon")){
                    lowmem_print(3, "FinalizerDaemon, not killed !!!!\n");
                    continue;
                }

                /* Exame QOS level of this task */
                if (oom_qos_endurance > 0 && oom_score_adj >= 470) {
                        get_monotonic_boottime(&delta);
                        delta = timespec_sub(delta, ns_to_timespec(p->real_start_time));
                        task_life_time = timespec_to_jiffies(&delta);
                        if (task_life_time < oom_qos_endurance*HZ) {
                                lowmem_print(3, "skip '%s' (%d), adj %hd, for QOS. lifeTime(ms) %u, \n",
                                             p->comm, p->pid, oom_score_adj,
                                             jiffies_to_msecs(task_life_time));
                                /* This is a candicate for killing. */
                                if (selected_qos) {
                                        if (oom_score_adj < selected_qos_oom_score_adj)
                                                continue;
                                        if (oom_score_adj == selected_qos_oom_score_adj &&
                                            tasksize <= selected_qos_tasksize) {
                                                continue;
                                        }
                                }
                                selected_qos = p;
                                selected_qos_tasksize = tasksize;
                                selected_qos_oom_score_adj = oom_score_adj;
                                continue;
                        }
                }

                if (selected) {
                        if (oom_score_adj < selected_oom_score_adj)
                                continue;
                        //For foreground and previous app, choice app allocating lower memory
                        if (oom_score_adj == 0 || oom_score_adj == 411) {
                                if (oom_score_adj == selected_oom_score_adj &&
                                    tasksize > selected_tasksize)
                                        continue;
                        } else {
                                if (oom_score_adj == selected_oom_score_adj) {
                                        /* the same adj level */
                                        if (selected_mark_as_prefer_kill &&
                                            !oom_qos_prefer_kill)
                                                continue;
                                        if (tasksize <= selected_tasksize) {
                                                continue;
                                        }
                                }
                        }
                }

#ifdef CONFIG_MTK_GMO_RAM_OPTIMIZE
		/*
		* if cached > 30MB, don't kill ub:secureRandom while its adj is 9
		*/
		if (!strcmp(p->comm, "ub:secureRandom") &&
			(REVERT_ADJ(oom_score_adj) == 9) && (other_file > 30*256)) {
			pr_info("select but ignore '%s' (%d), oom_score_adj %d, oom_adj %d, size %d, to kill, cache %ldkB is below limit %ldkB",
							p->comm, p->pid,
							oom_score_adj, REVERT_ADJ(oom_score_adj),
							tasksize,
							other_file * (long)(PAGE_SIZE / 1024),
							minfree * (long)(PAGE_SIZE / 1024));
		    continue;
		}
#endif

		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
                selected_mark_as_prefer_kill = oom_qos_prefer_kill;
		lowmem_print(2, "select '%s' (%d), adj %d, score_adj %hd, size %d, to kill\n",
			     p->comm, p->pid, REVERT_ADJ(oom_score_adj), oom_score_adj, tasksize);
	}

        /* make a decision between selected and selected_qos */
        if (selected && selected_qos && selected_oom_score_adj < 470) {
             /* Previous app or more critical */
             if (selected_qos_oom_score_adj > selected_oom_score_adj ||
                     (selected_qos_oom_score_adj == selected_oom_score_adj &&
                     selected_qos_tasksize > selected_tasksize)) {
                 selected = selected_qos;
                 selected_tasksize = selected_qos_tasksize;
                 selected_oom_score_adj = selected_qos_oom_score_adj;
             }
         }
#ifdef CONFIG_MT_ENG_BUILD
	if (log_offset > 0)
		lowmem_print(1, "\n%s", lmk_log_buf);
#endif

	if (selected) {
		long cache_size = other_file * (long)(PAGE_SIZE / 1024);
		long cache_limit = minfree * (long)(PAGE_SIZE / 1024);
		long free = other_free * (long)(PAGE_SIZE / 1024);
		trace_lowmemory_kill(selected, cache_size, cache_limit, free);
               if (selected_oom_score_adj <= 0)
                {
                    lowmem_print(1,"dump_timeout: %lu , jiffies: %lu\n",dump_meminfo_timeout,jiffies);
                    if (!time_before_eq(jiffies, dump_meminfo_timeout) || dump_meminfo_timeout==0)
                    {
                        dump_meminfo_timeout = jiffies + 5*HZ;
                        queue_work(dumpmem_workqueue, &dumpmem_work);
                    }
                }
		lowmem_print(1, "Killing '%s' (%d), adj %d, score_adj %hd,\n"
				"   to free %ldkB on behalf of '%s' (%d) because\n"
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n"
				"   Free memory is %ldkB above reserved, Force %d\n",
			     selected->comm, selected->pid,
				 REVERT_ADJ(selected_oom_score_adj),
			     selected_oom_score_adj,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     cache_size, cache_limit,
			     min_score_adj,
			     free,force_select);
		lowmem_deathpending = selected;
		lowmem_deathpending_timeout = jiffies + HZ;
		set_tsk_thread_flag(selected, TIF_MEMDIE);

		if (output_expect(enable_candidate_log)) {
			if (print_extra_info) {
				show_free_areas(0);
			#ifdef CONFIG_MTK_ION
				/* Show ION status */
				ion_mm_heap_memory_detail();
			#endif
			#ifdef CONFIG_MTK_GPU_SUPPORT
				if (mtk_dump_gpu_memory_usage() == false)
					lowmem_print(1, "mtk_dump_gpu_memory_usage not support\n");
			#endif
			}
		}

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
		/*
		* when kill adj=0 process trigger kernel warning, only in MTK internal eng load
		*/
		if ((selected_oom_score_adj <= lowmem_kernel_warn_adj) && /*lowmem_kernel_warn_adj=16 for test*/
			time_after_eq(jiffies, flm_warn_timeout)) {
			if (pid_dump != pid_flm_warn) {
				#define MSG_SIZE_TO_AEE 70
				char msg_to_aee[MSG_SIZE_TO_AEE];

				lowmem_print(1, "low memory trigger kernel warning\n");
				snprintf(msg_to_aee, MSG_SIZE_TO_AEE,
						"please contact AP/AF memory module owner[pid:%d]\n", pid_dump);
				aee_kernel_warning_api("LMK", 0, DB_OPT_DEFAULT |
					DB_OPT_DUMPSYS_ACTIVITY |
					DB_OPT_LOW_MEMORY_KILLER |
					DB_OPT_PID_MEMORY_INFO | /* smaps and hprof*/
					DB_OPT_PROCESS_COREDUMP |
					DB_OPT_DUMPSYS_SURFACEFLINGER |
					DB_OPT_DUMPSYS_GFXINFO |
					DB_OPT_DUMPSYS_PROCSTATS,
					"Framework low memory\nCRDISPATCH_KEY:FLM_APAF", msg_to_aee);

				if (pid_dump == selected->pid) {/*select 1st time, filter it*/
					/* pid_dump = pid_sec_mem; */
					pid_flm_warn = pid_dump;
					flm_warn_timeout = jiffies + 60*HZ;
					lowmem_deathpending = NULL;
					lowmem_print(1, "'%s' (%d) max RSS, not kill\n",
								selected->comm, selected->pid);
					send_sig(SIGSTOP, selected, 0);
					rcu_read_unlock();
					spin_unlock(&lowmem_shrink_lock);
					return rem;
				}
			} else {
				lowmem_print(1, "pid_flm_warn:%d, select '%s' (%d)\n",
								pid_flm_warn, selected->comm, selected->pid);
				pid_flm_warn = -1; /*reset*/
			}
		}
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
		/*
		* show an indication if low memory
		*/
		if (!in_lowmem && selected_oom_score_adj <= lowmem_debug_adj) {
			in_lowmem = 1;
			/* DAL_LowMemoryOn();*/
			lowmem_print(1, "LowMemoryOn\n");
			/* aee_kernel_warning(module_name, lowmem_warning);*/
		}
#endif

		send_sig(SIGKILL, selected, 0);
		rem += selected_tasksize;
                g_lmk_profile.select++;
	}

	lowmem_print(4, "lowmem_scan %lu, %x, return %lu\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	rcu_read_unlock();
        g_lmk_profile.reason_nop++;
        /* The highest value of task_bitmap is 'task_bitmap_mask',
         * means all slots are not empty.
         * So, we don't need to update timestamps. */
        if ((task_bitmap & task_bitmap_mask) >= task_bitmap_mask) {
                spin_unlock(&lowmem_shrink_lock);
                return rem;
        }

        /* update timestaps(future time) to record the empty slot */
        mutex_lock(&profile_mutex);
        empty_slot = task_bitmap | ~(task_bitmap_mask);
        jiffies_empty_slot = jiffies + msecs_to_jiffies(100);
        mutex_unlock(&profile_mutex);
        lowmem_print(4, "task_bitmap = %x\n", task_bitmap);
	spin_unlock(&lowmem_shrink_lock);
	return rem;
}
static int lmk_proc_show(struct seq_file *m, void *v)
{
    unsigned long p;

    mutex_lock(&profile_mutex);
    /* out of date? */
    if (time_after(jiffies, g_lmk_profile.timeout + HZ) ||
        g_lmk_profile.timeout == INITIAL_JIFFIES) {
        p = 0;
    } else {
        p = g_lmk_profile.pressure;
        standardize_lmk_pressure(p);
    }
    mutex_unlock(&profile_mutex);
    seq_printf(m, "pressure:       %8lu\n", p);
    seq_printf(m, "reserve_KB:     %8lu\n", totalreserve_pages*(PAGE_SIZE / 1024));
    return 0;
}

static int lmk_proc_open(struct inode *inode, struct file *file)
{
       return single_open(file, lmk_proc_show, NULL);
}

static const struct file_operations lmk_proc_fops = {
    .owner = THIS_MODULE,
    .open = lmk_proc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static struct shrinker lowmem_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
#ifdef CONFIG_HIGHMEM
	unsigned long normal_pages;
#endif

#ifdef CONFIG_ZRAM
	vm_swappiness = 100;
#endif


	task_free_register(&task_nb);
	register_shrinker(&lowmem_shrinker);
        dumpmem_workqueue = create_workqueue("dumpmem_wq");
//        proc_create("lmkinfo", 0444, NULL, &lmk_proc_fops);
#ifdef CONFIG_HIGHMEM
	normal_pages = totalram_pages - totalhigh_pages;
	total_low_ratio = (totalram_pages + normal_pages - 1) / normal_pages;
	pr_err("[LMK]total_low_ratio[%d] - totalram_pages[%lu] - totalhigh_pages[%lu]\n",
			total_low_ratio, totalram_pages, totalhigh_pages);
#endif

	return 0;
}

static void __exit lowmem_exit(void)
{
        destroy_workqueue(dumpmem_workqueue);
	unregister_shrinker(&lowmem_shrinker);
	task_free_unregister(&task_nb);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

/*
 * get_min_free_pages
 * returns the low memory killer watermark of the given pid,
 * When the system free memory is lower than the watermark, the LMK (low memory
 * killer) may try to kill processes.
 */
int get_min_free_pages(pid_t pid)
{
	struct task_struct *p;
	int target_oom_adj = 0;
	int i = 0;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

	for_each_process(p) {
		/* search pid */
		if (p->pid == pid) {
			task_lock(p);
			target_oom_adj = p->signal->oom_score_adj;
			task_unlock(p);
			/* get min_free value of the pid */
			for (i = array_size - 1; i >= 0; i--) {
				if (target_oom_adj >= lowmem_adj[i]) {
					pr_debug("pid: %d, target_oom_adj = %d, lowmem_adj[%d] = %d, lowmem_minfree[%d] = %d\n",
							pid, target_oom_adj, i, lowmem_adj[i], i,
							lowmem_minfree[i]);
					return lowmem_minfree[i];
				}
			}
			goto out;
		}
	}

out:
	lowmem_print(3, "[%s]pid: %d, adj: %d, lowmem_minfree = 0\n",
			__func__, pid, p->signal->oom_score_adj);
	return 0;
}
EXPORT_SYMBOL(get_min_free_pages);

/* Query LMK minfree settings */
/* To query default value, you can input index with value -1. */
size_t query_lmk_minfree(int index)
{
	int which;

	/* Invalid input index, return default value */
	if (index < 0)
		return lowmem_minfree[2];

	/* Find a corresponding output */
	which = 5;
	do {
		if (lowmem_adj[which] <= index)
			break;
	} while (--which >= 0);

	/* Fix underflow bug */
	which = (which < 0) ? 0 : which;

	return lowmem_minfree[which];
}
EXPORT_SYMBOL(query_lmk_minfree);

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
module_param_cb(adj, &lowmem_adj_array_ops,
		.arr = &__param_arr_adj, S_IRUGO | S_IWUSR);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static int debug_adj_set(const char *val, const struct kernel_param *kp)
{
	const int ret = param_set_uint(val, kp);

	lowmem_debug_adj = lowmem_oom_adj_to_oom_score_adj(lowmem_debug_adj);
	return ret;
}

static struct kernel_param_ops debug_adj_ops = {
	.set = &debug_adj_set,
	.get = &param_get_uint,
};

module_param_cb(debug_adj, &debug_adj_ops, &lowmem_debug_adj, S_IRUGO | S_IWUSR);
__MODULE_PARM_TYPE(debug_adj, short);

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MT_ENG_BUILD)
static int flm_warn_adj_set(const char *val, const struct kernel_param *kp)
{
	const int ret = param_set_uint(val, kp);

	lowmem_kernel_warn_adj = lowmem_oom_adj_to_oom_score_adj(lowmem_kernel_warn_adj);
	return ret;
}

static struct kernel_param_ops flm_warn_adj_ops = {
	.set = &flm_warn_adj_set,
	.get = &param_get_uint,
};
module_param_cb(flm_warn_adj, &flm_warn_adj_ops, &lowmem_kernel_warn_adj, S_IRUGO | S_IWUSR);
#endif
#else
module_param_named(debug_adj, lowmem_debug_adj, short, S_IRUGO | S_IWUSR);
#endif
module_param_named(candidate_log, enable_candidate_log, uint, S_IRUGO | S_IWUSR);

late_initcall(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

