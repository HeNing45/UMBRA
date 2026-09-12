/*
 * UMBRA bare-metal CoreMark platform configuration.
 *
 * CoreMark upstream is Apache-2.0 licensed. This port file is intentionally
 * separate from the protected benchmark sources and is the only C-level
 * platform contract consumed by them.
 */
#ifndef UMBRA_CORE_PORTME_H
#define UMBRA_CORE_PORTME_H

#include <stddef.h>
#include <stdint.h>

#define HAS_FLOAT  0
#define HAS_TIME_H 0
#define USE_CLOCK  0
#define HAS_STDIO  0
#define HAS_PRINTF 0

#ifndef COMPILER_VERSION
#define COMPILER_VERSION "xPack GCC " __VERSION__
#endif
#ifndef FLAGS_STR
#define FLAGS_STR "flags unavailable"
#endif
#ifndef COMPILER_FLAGS
#define COMPILER_FLAGS FLAGS_STR
#endif
#ifndef MEM_LOCATION
#define MEM_LOCATION "RTL zero-latency 256 KiB memory model"
#endif

typedef int16_t   ee_s16;
typedef uint16_t  ee_u16;
typedef int32_t   ee_s32;
typedef float     ee_f32;
typedef uint8_t   ee_u8;
typedef uint32_t  ee_u32;
typedef uintptr_t ee_ptr_int;
typedef size_t    ee_size_t;

#define align_mem(x) (void *)(4u + (((ee_ptr_int)(x)-1u) & ~(ee_ptr_int)3u))

#define CORETIMETYPE ee_u32
typedef ee_u32 CORE_TICKS;

#ifndef SEED_METHOD
#define SEED_METHOD SEED_VOLATILE
#endif
#ifndef MEM_METHOD
#define MEM_METHOD MEM_STATIC
#endif

#ifndef MULTITHREAD
#define MULTITHREAD 1
#endif
#define USE_PTHREAD 0
#define USE_FORK    0
#define USE_SOCKET  0

#define MAIN_HAS_NOARGC 1
#define MAIN_HAS_NORETURN 0

extern ee_u32 default_num_contexts;

typedef struct CORE_PORTABLE_S
{
    ee_u8 portable_id;
} core_portable;

void portable_init(core_portable *p, int *argc, char *argv[]);
void portable_fini(core_portable *p);

#if !defined(PROFILE_RUN) && !defined(PERFORMANCE_RUN) \
    && !defined(VALIDATION_RUN)
#error "Build must select PERFORMANCE_RUN, VALIDATION_RUN, or PROFILE_RUN"
#endif

int ee_printf(const char *fmt, ...);

#endif
