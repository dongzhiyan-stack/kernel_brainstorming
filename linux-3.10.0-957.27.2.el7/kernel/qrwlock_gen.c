/*
 * Copyright (2004) Linus Torvalds
 *
 * Author: Zwane Mwaikambo <zwane@fsmlabs.com>
 *
 * Copyright (2004, 2005) Ingo Molnar
 *
 * This file contains the spinlock/qrwlock implementations for the
 * SMP and the DEBUG_SPINLOCK cases. (UP-nondebug inlines them)
 *
 * Note that some architectures have special knowledge about the
 * stack frames of these functions in their profile_pc. If you
 * change anything significant here that could change the stack
 * frame contact the architecture maintainers.
 */

#include <linux/linkage.h>
#include <linux/preempt.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/debug_locks.h>
#include <linux/export.h>

/*
 * If lockdep is enabled then we use the non-preemption spin-ops
 * even on CONFIG_PREEMPT, because lockdep assumes that interrupts are
 * not re-enabled during lock-acquire (which the preempt-spin-ops do):
 */
#if !defined(CONFIG_GENERIC_LOCKBREAK) || defined(CONFIG_DEBUG_LOCK_ALLOC)
/*
 * The __lock_function inlines are taken from
 * include/linux/spinlock_api_smp.h
 */
#else
#define raw_qread_can_lock(l)	qread_can_lock(l)
#define raw_qwrite_can_lock(l)	qwrite_can_lock(l)
/*
 * We build the __lock_function inlines here. They are too large for
 * inlining all over the place, but here is only one user per function
 * which embedds them into the calling _lock_function below.
 *
 * This could be a long-held lock. We both prepare to spin for a long
 * time (making _this_ CPU preemptable if possible), and we also signal
 * towards that other CPU that it should break the lock ASAP.
 */
#define BUILD_LOCK_OPS(op, locktype)					\
void __lockfunc __raw_##op##_lock(locktype##_t *lock)			\
{									\
	for (;;) {							\
		preempt_disable();					\
		if (likely(do_raw_##op##_trylock(lock)))		\
			break;						\
		preempt_enable();					\
									\
		if (!(lock)->break_lock)				\
			(lock)->break_lock = 1;				\
		while (!raw_##op##_can_lock(lock) && (lock)->break_lock)\
			arch_##op##_relax(&lock->raw_lock);		\
	}								\
	(lock)->break_lock = 0;						\
}									\
									\
unsigned long __lockfunc __raw_##op##_lock_irqsave(locktype##_t *lock)	\
{									\
	unsigned long flags;						\
									\
	for (;;) {							\
		preempt_disable();					\
		local_irq_save(flags);					\
		if (likely(do_raw_##op##_trylock(lock)))		\
			break;						\
		local_irq_restore(flags);				\
		preempt_enable();					\
									\
		if (!(lock)->break_lock)				\
			(lock)->break_lock = 1;				\
		while (!raw_##op##_can_lock(lock) && (lock)->break_lock)\
			arch_##op##_relax(&lock->raw_lock);		\
	}								\
	(lock)->break_lock = 0;						\
	return flags;							\
}									\
									\
void __lockfunc __raw_##op##_lock_irq(locktype##_t *lock)		\
{									\
	_raw_##op##_lock_irqsave(lock);					\
}									\
									\
void __lockfunc __raw_##op##_lock_bh(locktype##_t *lock)		\
{									\
	unsigned long flags;						\
									\
	/*							*/	\
	/* Careful: we must exclude softirqs too, hence the	*/	\
	/* irq-disabling. We use the generic preemption-aware	*/	\
	/* function:						*/	\
	/**/								\
	flags = _raw_##op##_lock_irqsave(lock);				\
	local_bh_disable();						\
	local_irq_restore(flags);					\
}									\

/*
 * Build preemption-friendly versions of the following
 * lock-spinning functions:
 *
 *         __[spin|qread|qwrite]_lock()
 *         __[spin|qread|qwrite]_lock_irq()
 *         __[spin|qread|qwrite]_lock_irqsave()
 *         __[spin|qread|qwrite]_lock_bh()
 */
BUILD_LOCK_OPS(qread, qrwlock);
BUILD_LOCK_OPS(qwrite, qrwlock);

#endif


/* BEGIN_RWLOCK */
#ifndef CONFIG_INLINE_READ_TRYLOCK
int __lockfunc _raw_qread_trylock(qrwlock_t *lock)
{
	return __raw_qread_trylock(lock);
}
EXPORT_SYMBOL(_raw_qread_trylock);
#endif

#ifndef CONFIG_INLINE_READ_LOCK
void __lockfunc _raw_qread_lock(qrwlock_t *lock)
{
	__raw_qread_lock(lock);
}
EXPORT_SYMBOL(_raw_qread_lock);
#endif

#ifndef CONFIG_INLINE_READ_LOCK_IRQSAVE
unsigned long __lockfunc _raw_qread_lock_irqsave(qrwlock_t *lock)
{
	return __raw_qread_lock_irqsave(lock);
}
EXPORT_SYMBOL(_raw_qread_lock_irqsave);
#endif

#ifndef CONFIG_INLINE_READ_LOCK_IRQ
void __lockfunc _raw_qread_lock_irq(qrwlock_t *lock)
{
	__raw_qread_lock_irq(lock);
}
EXPORT_SYMBOL(_raw_qread_lock_irq);
#endif

#ifndef CONFIG_INLINE_READ_LOCK_BH
void __lockfunc _raw_qread_lock_bh(qrwlock_t *lock)
{
	__raw_qread_lock_bh(lock);
}
EXPORT_SYMBOL(_raw_qread_lock_bh);
#endif

#ifndef CONFIG_INLINE_READ_UNLOCK
void __lockfunc _raw_qread_unlock(qrwlock_t *lock)
{
	__raw_qread_unlock(lock);
}
EXPORT_SYMBOL(_raw_qread_unlock);
#endif

#ifndef CONFIG_INLINE_READ_UNLOCK_IRQRESTORE
void __lockfunc _raw_qread_unlock_irqrestore(qrwlock_t *lock, unsigned long flags)
{
	__raw_qread_unlock_irqrestore(lock, flags);
}
EXPORT_SYMBOL(_raw_qread_unlock_irqrestore);
#endif

#ifndef CONFIG_INLINE_READ_UNLOCK_IRQ
void __lockfunc _raw_qread_unlock_irq(qrwlock_t *lock)
{
	__raw_qread_unlock_irq(lock);
}
EXPORT_SYMBOL(_raw_qread_unlock_irq);
#endif

#ifndef CONFIG_INLINE_READ_UNLOCK_BH
void __lockfunc _raw_qread_unlock_bh(qrwlock_t *lock)
{
	__raw_qread_unlock_bh(lock);
}
EXPORT_SYMBOL(_raw_qread_unlock_bh);
#endif

#ifndef CONFIG_INLINE_WRITE_TRYLOCK
int __lockfunc _raw_qwrite_trylock(qrwlock_t *lock)
{
	return __raw_qwrite_trylock(lock);
}
EXPORT_SYMBOL(_raw_qwrite_trylock);
#endif

#ifndef CONFIG_INLINE_WRITE_LOCK
void __lockfunc _raw_qwrite_lock(qrwlock_t *lock)
{
	__raw_qwrite_lock(lock);
}
EXPORT_SYMBOL(_raw_qwrite_lock);
#endif

#ifndef CONFIG_INLINE_WRITE_LOCK_IRQSAVE
unsigned long __lockfunc _raw_qwrite_lock_irqsave(qrwlock_t *lock)
{
	return __raw_qwrite_lock_irqsave(lock);
}
EXPORT_SYMBOL(_raw_qwrite_lock_irqsave);
#endif

#ifndef CONFIG_INLINE_WRITE_LOCK_IRQ
void __lockfunc _raw_qwrite_lock_irq(qrwlock_t *lock)
{
	__raw_qwrite_lock_irq(lock);
}
EXPORT_SYMBOL(_raw_qwrite_lock_irq);
#endif

#ifndef CONFIG_INLINE_WRITE_LOCK_BH
void __lockfunc _raw_qwrite_lock_bh(qrwlock_t *lock)
{
	__raw_qwrite_lock_bh(lock);
}
EXPORT_SYMBOL(_raw_qwrite_lock_bh);
#endif

#ifndef CONFIG_INLINE_WRITE_UNLOCK
void __lockfunc _raw_qwrite_unlock(qrwlock_t *lock)
{
	__raw_qwrite_unlock(lock);
}
EXPORT_SYMBOL(_raw_qwrite_unlock);
#endif

#ifndef CONFIG_INLINE_WRITE_UNLOCK_IRQRESTORE
void __lockfunc _raw_qwrite_unlock_irqrestore(qrwlock_t *lock, unsigned long flags)
{
	__raw_qwrite_unlock_irqrestore(lock, flags);
}
EXPORT_SYMBOL(_raw_qwrite_unlock_irqrestore);
#endif

#ifndef CONFIG_INLINE_WRITE_UNLOCK_IRQ
void __lockfunc _raw_qwrite_unlock_irq(qrwlock_t *lock)
{
	__raw_qwrite_unlock_irq(lock);
}
EXPORT_SYMBOL(_raw_qwrite_unlock_irq);
#endif

#ifndef CONFIG_INLINE_WRITE_UNLOCK_BH
void __lockfunc _raw_qwrite_unlock_bh(qrwlock_t *lock)
{
	__raw_qwrite_unlock_bh(lock);
}
EXPORT_SYMBOL(_raw_qwrite_unlock_bh);
#endif
/* END_RWLOCK */

