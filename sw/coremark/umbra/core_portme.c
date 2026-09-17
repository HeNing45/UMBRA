/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: Apache-2.0
 */

/* UMBRA simulation-only CoreMark platform implementation. */
#include "coremark.h"
#include "core_portme.h"

#include <stdarg.h>

#define UMBRA_MMIO_CYCLE (*(volatile ee_u32 *)0x0003ff00u)
#define UMBRA_MMIO_UART  (*(volatile ee_u32 *)0x0003ff04u)
#define UMBRA_MMIO_DONE  (*(volatile ee_u32 *)0x0003ff08u)

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

static CORETIMETYPE start_time_val;
static CORETIMETYPE stop_time_val;

static CORETIMETYPE
umbra_cycle(void)
{
    return UMBRA_MMIO_CYCLE;
}

void
start_time(void)
{
    start_time_val = umbra_cycle();
}

void
stop_time(void)
{
    stop_time_val = umbra_cycle();
}

CORE_TICKS
get_time(void)
{
    return (CORE_TICKS)(stop_time_val - start_time_val);
}

secs_ret
time_in_secs(CORE_TICKS ticks)
{
    /*
 * The RTL cycle counter does not establish physical execution time.
 * Returning zero deliberately prevents the upstream benchmark from
 * presenting a short simulation as a reportable >=10-second result.
 * The runner reports raw cycles and iterations/Mcycle instead.
 */
    (void)ticks;
    return (secs_ret)0;
}

ee_u32 default_num_contexts = 1;

void
portable_init(core_portable *p, int *argc, char *argv[])
{
    (void)argc;
    (void)argv;
    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
        ee_printf("ERROR! ee_ptr_int cannot hold a pointer\n");
    if (sizeof(ee_u32) != 4u)
        ee_printf("ERROR! ee_u32 is not 32 bits\n");
    p->portable_id = 1;
}

#if defined(UMBRA_GCOV_DUMP)
/*
 * Bare-metal profile extraction for PGO (CoreMark run rule 3).
 *
 * There is no filesystem, so libgcov cannot write .gcda itself. Instead
 * -fprofile-info-section places one gcov_info pointer per translation unit in
 * .gcov_info, and __gcov_info_to_gcda streams the equivalent bytes through the
 * callbacks below. We frame them as hex on the simulation UART; the host
 * decodes the stream and feeds it to `gcov-tool merge-stream`, which is the
 * documented cross-profiling path. Only core_portme.c and the linker script
 * are touched, both of which the CoreMark run rules place under "Allowed".
 */
extern const void *__gcov_info_start[];
extern const void *__gcov_info_end[];

extern void __gcov_info_to_gcda(const void *,
                                void (*)(const char *, void *),
                                void (*)(const void *, unsigned, void *),
                                void *(*)(unsigned, void *),
                                void *);
extern void __gcov_filename_to_gcfn(const char *,
                                    void (*)(const void *, unsigned, void *),
                                    void *);

/*
 * Freestanding stubs for the libc symbols libgcov references. Only linked
 * into the instrumented build. calloc is genuinely used at run time by the
 * indirect-call/topn value profilers (CoreMark's list comparator is an
 * indirect call), so it is backed by the arena rather than stubbed out;
 * fread is referenced only from the file-reading merge path we never enter.
 */
static unsigned char umbra_gcov_arena[32768];
static unsigned       umbra_gcov_used = 0u;

static void *
umbra_gcov_arena_alloc(unsigned length)
{
    unsigned aligned = (length + 7u) & ~7u;
    if (umbra_gcov_used + aligned > sizeof(umbra_gcov_arena))
        return (void *)0;
    void *p = &umbra_gcov_arena[umbra_gcov_used];
    umbra_gcov_used += aligned;
    return p;
}

void *
calloc(unsigned nmemb, unsigned size)
{
    unsigned total = nmemb * size;
    unsigned char *p = (unsigned char *)umbra_gcov_arena_alloc(total);
    unsigned i;
    if (p)
        for (i = 0u; i < total; i++)
            p[i] = 0u;
    return p;
}

void *
malloc(unsigned size)
{
    return umbra_gcov_arena_alloc(size);
}

void
free(void *p)
{
    (void)p; /* arena is bump-only; the run ends immediately after the dump */
}

unsigned
strlen(const char *s)
{
    const char *e = s;
    while (*e)
        e++;
    return (unsigned)(e - s);
}

void *
memcpy(void *d, const void *s, unsigned n)
{
    unsigned char *dd = (unsigned char *)d;
    const unsigned char *ss = (const unsigned char *)s;
    unsigned i;
    for (i = 0u; i < n; i++)
        dd[i] = ss[i];
    return d;
}

void *
memset(void *d, int c, unsigned n)
{
    unsigned char *dd = (unsigned char *)d;
    unsigned i;
    for (i = 0u; i < n; i++)
        dd[i] = (unsigned char)c;
    return d;
}

unsigned
fread(void *p, unsigned sz, unsigned n, void *f)
{
    (void)p; (void)sz; (void)n; (void)f;
    return 0u; /* unreachable: the target never reads .gcda back */
}

void
abort(void)
{
    ee_printf("\nGCOVABORT\n");
    UMBRA_MMIO_DONE = 1u;
    for (;;)
        ;
}

static void *
umbra_gcov_allocate(unsigned length, void *arg)
{
    void *p;
    (void)arg;
    p = umbra_gcov_arena_alloc(length);
    if (!p)
        ee_printf("GCOVERR arena exhausted\n");
    return p;
}

static void
umbra_gcov_emit(const void *data, unsigned length, void *arg)
{
    static const char hex[] = "0123456789abcdef";
    const unsigned char *b = (const unsigned char *)data;
    static unsigned col = 0u;
    unsigned i;
    (void)arg;
    for (i = 0u; i < length; i++)
    {
        UMBRA_MMIO_UART = (ee_u32)hex[(b[i] >> 4) & 0xfu];
        UMBRA_MMIO_UART = (ee_u32)hex[b[i] & 0xfu];
        if (++col == 32u)
        {
            UMBRA_MMIO_UART = (ee_u32)'\n';
            col = 0u;
        }
    }
}

static void
umbra_gcov_filename(const char *name, void *arg)
{
    __gcov_filename_to_gcfn(name, umbra_gcov_emit, arg);
}

static void
umbra_gcov_dump(void)
{
    const void **info = (const void **)__gcov_info_start;
    const void **end  = (const void **)__gcov_info_end;

    ee_printf("\nGCOVSTART\n");
    while (info != end)
        __gcov_info_to_gcda(*info++, umbra_gcov_filename, umbra_gcov_emit,
                            umbra_gcov_allocate, (void *)0);
    ee_printf("\nGCOVEND\n");
}
#endif /* UMBRA_GCOV_DUMP */

void
portable_fini(core_portable *p)
{
    p->portable_id = 0;
#if defined(UMBRA_GCOV_DUMP)
    umbra_gcov_dump();
#endif
    UMBRA_MMIO_DONE = 1u;
}

void *
portable_malloc(ee_size_t size)
{
    (void)size;
    return (void *)0;
}

void
portable_free(void *p)
{
    (void)p;
}

static int
uart_putc(char c)
{
    UMBRA_MMIO_UART = (ee_u32)(ee_u8)c;
    return 1;
}

static int
print_unsigned(ee_u32 value, unsigned base, unsigned width, char pad, int upper)
{
    char digits_lower[] = "0123456789abcdef";
    char digits_upper[] = "0123456789ABCDEF";
    char buf[16];
    char *digits = upper ? digits_upper : digits_lower;
    unsigned used = 0;
    int count = 0;

    do
    {
        buf[used++] = digits[value % base];
        value /= base;
    } while (value != 0u);

    while (used < width)
    {
        count += uart_putc(pad);
        width--;
    }
    while (used != 0u)
        count += uart_putc(buf[--used]);
    return count;
}

int
ee_printf(const char *fmt, ...)
{
    va_list args;
    int count = 0;

    va_start(args, fmt);
    while (*fmt != '\0')
    {
        unsigned width = 0;
        char pad = ' ';
        int is_long = 0;

        if (*fmt != '%')
        {
            count += uart_putc(*fmt++);
            continue;
        }
        fmt++;
        if (*fmt == '%')
        {
            count += uart_putc(*fmt++);
            continue;
        }
        if (*fmt == '0')
        {
            pad = '0';
            fmt++;
        }
        while (*fmt >= '0' && *fmt <= '9')
        {
            width = width * 10u + (unsigned)(*fmt - '0');
            fmt++;
        }
        if (*fmt == 'l')
        {
            is_long = 1;
            fmt++;
        }

        switch (*fmt++)
        {
            case 'd':
            case 'i':
            {
                ee_s32 value = is_long ? (ee_s32)va_arg(args, long)
                                       : (ee_s32)va_arg(args, int);
                ee_u32 magnitude;
                if (value < 0)
                {
                    count += uart_putc('-');
                    if (width != 0u)
                        width--;
                    magnitude = (ee_u32)(-(value + 1)) + 1u;
                }
                else
                    magnitude = (ee_u32)value;
                count += print_unsigned(magnitude, 10u, width, pad, 0);
                break;
            }
            case 'u':
            {
                ee_u32 value = is_long ? (ee_u32)va_arg(args, unsigned long)
                                       : (ee_u32)va_arg(args, unsigned int);
                count += print_unsigned(value, 10u, width, pad, 0);
                break;
            }
            case 'x':
            case 'X':
            {
                char spec = fmt[-1];
                ee_u32 value = is_long ? (ee_u32)va_arg(args, unsigned long)
                                       : (ee_u32)va_arg(args, unsigned int);
                count += print_unsigned(value, 16u, width, pad, spec == 'X');
                break;
            }
            case 'c':
                count += uart_putc((char)va_arg(args, int));
                break;
            case 's':
            {
                const char *s = va_arg(args, const char *);
                if (s == (const char *)0)
                    s = "(null)";
                while (*s != '\0')
                    count += uart_putc(*s++);
                break;
            }
            default:
                count += uart_putc('?');
                break;
        }
    }
    va_end(args);
    return count;
}
