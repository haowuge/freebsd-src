#ifndef _MACHINE_BITS_H_
#define _MACHINE_BITS_H_

#ifdef __LP64__
#define BITS_PER_LONG		64
#define BITS_PER_LONG_LONG	64
#else
#define BITS_PER_LONG		32
#define BITS_PER_LONG_LONG	32
#endif

#define BIT_U(n) (1U << (n))
#define BIT_UL(nr)   (1UL << (nr))
#define BIT_ULL(nr)   (1ULL << (nr))

#define UL(x)		x##UL
#define ULL(x)	x##ULL

#define GENMASK_INPUT_CHECK(h, l)		0

#define __GENMASK(h, l) \
        (((~UL(0)) - (UL(1) << (l)) + 1) & \
         (~UL(0) >> (BITS_PER_LONG - 1 - (h))))
#define _GENMASK(h, l) \
        (GENMASK_INPUT_CHECK(h, l) + __GENMASK(h, l))

#define __GENMASK_ULL(h, l) \
        (((~ULL(0)) - (ULL(1) << (l)) + 1) & \
         (~ULL(0) >> (BITS_PER_LONG_LONG - 1 - (h))))
#define _GENMASK_ULL(h, l) \
        (GENMASK_INPUT_CHECK(h, l) + __GENMASK_ULL(h, l))

#endif
