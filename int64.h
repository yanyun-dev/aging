/****************************************************************************
 * @file     int64.h
 * @brief    Version number
 * @date     13 February 2015
 *
 * @note
 * Copyright (C) 2015, Active-Semi International
 *
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY ACTIVE-SEMI INTERNATIONAL;
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
 * ACTIVE-SEMICONDUCTOR IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 ******************************************************************************/
#ifndef __libfixmath_int64_h__
#define __libfixmath_int64_h__

#ifdef __cplusplus
extern "C"
{
#endif

#include "pac52xx.h"

#define FIXMATH_NO_64BIT

#ifndef FIXMATH_NO_64BIT
static inline  int64_t int64_const(int32_t hi, uint32_t lo) { return (((int64_t)hi << 32) | lo); }
static inline  int64_t int64_from_int32(int32_t x) { return (int64_t)x; }
static inline  int32_t int64_hi(int64_t x) { return (x >> 32); }
static inline uint32_t int64_lo(int64_t x) { return (x & ((1ULL << 32) - 1)); }

static inline int64_t int64_add(int64_t x, int64_t y)   { return (x + y);  }
static inline int64_t int64_neg(int64_t x)              { return (-x);     }
static inline int64_t int64_sub(int64_t x, int64_t y)   { return (x - y);  }
static inline int64_t int64_shift(int64_t x, int8_t y)  { return (y < 0 ? (x >> -y) : (x << y)); }

static inline int64_t int64_mul_i32_i32(int32_t x, int32_t y) { return (x * y);  }
static inline int64_t int64_mul_i64_i32(int64_t x, int32_t y) { return (x * y);  }

static inline int64_t int64_div_i64_i32(int64_t x, int32_t y) { return (x / y);  }

static inline int int64_cmp_eq(int64_t x, int64_t y) { return (x == y); }
static inline int int64_cmp_ne(int64_t x, int64_t y) { return (x != y); }
static inline int int64_cmp_gt(int64_t x, int64_t y) { return (x >  y); }
static inline int int64_cmp_ge(int64_t x, int64_t y) { return (x >= y); }
static inline int int64_cmp_lt(int64_t x, int64_t y) { return (x <  y); }
static inline int int64_cmp_le(int64_t x, int64_t y) { return (x <= y); }
#else

typedef struct {
         int32_t hi;
        uint32_t lo;
} __int64_t;

#if 0
PAC5XXX_RAMFUNC static inline __int64_t int64_const(int32_t hi, uint32_t lo) { return (__int64_t){ hi, lo }; }
PAC5XXX_RAMFUNC static inline __int64_t int64_from_int32(int32_t x) { return (__int64_t){ (x < 0 ? -1 : 0), x }; }
PAC5XXX_RAMFUNC static inline   int32_t int64_hi(__int64_t x) { return x.hi; }
PAC5XXX_RAMFUNC static inline  uint32_t int64_lo(__int64_t x) { return x.lo; }

PAC5XXX_RAMFUNC static inline int int64_cmp_eq(__int64_t x, __int64_t y) { return ((x.hi == y.hi) && (x.lo == y.lo)); }
PAC5XXX_RAMFUNC static inline int int64_cmp_ne(__int64_t x, __int64_t y) { return ((x.hi != y.hi) || (x.lo != y.lo)); }
PAC5XXX_RAMFUNC static inline int int64_cmp_gt(__int64_t x, __int64_t y) { return ((x.hi > y.hi) || ((x.hi == y.hi) && (x.lo >  y.lo))); }
PAC5XXX_RAMFUNC static inline int int64_cmp_ge(__int64_t x, __int64_t y) { return ((x.hi > y.hi) || ((x.hi == y.hi) && (x.lo >= y.lo))); }
PAC5XXX_RAMFUNC static inline int int64_cmp_lt(__int64_t x, __int64_t y) { return ((x.hi < y.hi) || ((x.hi == y.hi) && (x.lo <  y.lo))); }
PAC5XXX_RAMFUNC static inline int int64_cmp_le(__int64_t x, __int64_t y) { return ((x.hi < y.hi) || ((x.hi == y.hi) && (x.lo <= y.lo))); }

extern PAC5XXX_RAMFUNC __int64_t int64_add(__int64_t x, __int64_t y);

PAC5XXX_RAMFUNC __int64_t int64_neg(__int64_t x);

PAC5XXX_RAMFUNC __int64_t int64_sub(__int64_t x, __int64_t y);

PAC5XXX_RAMFUNC __int64_t int64_shift(__int64_t x, int8_t y);

PAC5XXX_RAMFUNC __int64_t int64_mul_i32_i32(int32_t x, int32_t y);

PAC5XXX_RAMFUNC __int64_t int64_mul_i64_i32(__int64_t x, int32_t y);

PAC5XXX_RAMFUNC __int64_t int64_div_i64_i32(__int64_t x, int32_t y);

#else

static inline __int64_t int64_const(int32_t hi, uint32_t lo) { return (__int64_t){ hi, lo }; }
static inline __int64_t int64_from_int32(int32_t x) { return (__int64_t){ (x < 0 ? -1 : 0), x }; }
static inline   int32_t int64_hi(__int64_t x) { return x.hi; }
static inline  uint32_t int64_lo(__int64_t x) { return x.lo; }

static inline int int64_cmp_eq(__int64_t x, __int64_t y) { return ((x.hi == y.hi) && (x.lo == y.lo)); }
static inline int int64_cmp_ne(__int64_t x, __int64_t y) { return ((x.hi != y.hi) || (x.lo != y.lo)); }
static inline int int64_cmp_gt(__int64_t x, __int64_t y) { return ((x.hi > y.hi) || ((x.hi == y.hi) && (x.lo >  y.lo))); }
static inline int int64_cmp_ge(__int64_t x, __int64_t y) { return ((x.hi > y.hi) || ((x.hi == y.hi) && (x.lo >= y.lo))); }
static inline int int64_cmp_lt(__int64_t x, __int64_t y) { return ((x.hi < y.hi) || ((x.hi == y.hi) && (x.lo <  y.lo))); }
static inline int int64_cmp_le(__int64_t x, __int64_t y) { return ((x.hi < y.hi) || ((x.hi == y.hi) && (x.lo <= y.lo))); }

extern __int64_t int64_add(__int64_t x, __int64_t y);

__int64_t int64_neg(__int64_t x);

__int64_t int64_sub(__int64_t x, __int64_t y);

__int64_t int64_shift(__int64_t x, int8_t y);

__int64_t int64_mul_i32_i32(int32_t x, int32_t y);

__int64_t int64_mul_i64_i32(__int64_t x, int32_t y);

__int64_t int64_div_i64_i32(__int64_t x, int32_t y);

#endif





#define int64_t __int64_t

#endif

#ifdef __cplusplus
}
#endif

#endif
