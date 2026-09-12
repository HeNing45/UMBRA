/* UMBRA Embench-IoT board support.
 *
 * Textually included by upstream support/board.c (never compiled standalone).
 * Platform seam is the same simulation-only MMIO triple as the CoreMark port:
 * 0x0003ff00 read - low 32 bits of the RTL cycle counter
 * 0x0003ff04 write - one output character in wdata[7:0]
 * 0x0003ff08 write - done value (written by start.S from main's return)
 *
 * start_trigger/stop_trigger sample the cycle counter around the timed
 * benchmark call; stop_trigger prints the delta so the runner can parse
 * one canonical "EMBENCH_TICKS <n>" line per run. Embench measures time at a
 * declared clock; on a cycle-accurate simulator ticks at 1 MHz equivalence
 * give ms = ticks / 1000.
 */

#include "support.h"

#include <stddef.h>

#define UMBRA_MMIO_CYCLE (*(volatile unsigned int *) 0x0003ff00u)
#define UMBRA_MMIO_UART  (*(volatile unsigned int *) 0x0003ff04u)

static unsigned int umbra_start_ticks;
static unsigned int umbra_stop_ticks;

static void
umbra_putc (char c)
{
  UMBRA_MMIO_UART = (unsigned int) (unsigned char) c;
}

static void
umbra_puts (const char *s)
{
  while (*s != '\0')
    umbra_putc (*s++);
}

static void
umbra_putdec (unsigned int value)
{
  char buf[10];
  unsigned int used = 0;

  do
    {
      buf[used++] = (char) ('0' + (value % 10u));
      value /= 10u;
    }
  while (value != 0u);

  while (used != 0u)
    umbra_putc (buf[--used]);
}

void
initialise_board (void)
{
}

void __attribute__ ((noinline))
start_trigger (void)
{
  umbra_start_ticks = UMBRA_MMIO_CYCLE;
}

void __attribute__ ((noinline))
stop_trigger (void)
{
  umbra_stop_ticks = UMBRA_MMIO_CYCLE;
  umbra_puts ("EMBENCH_TICKS ");
  umbra_putdec (umbra_stop_ticks - umbra_start_ticks);
  umbra_putc ('\n');
}

/* Freestanding string/memory routines. GCC emits calls to these for
 * aggregate copies and cleared arrays even under -ffreestanding, and several
 * benchmarks call them by name. Weak so any benchmark-local definition wins.
 */

void *__attribute__ ((weak))
memcpy (void *dest, const void *src, size_t n)
{
  unsigned char *d = (unsigned char *) dest;
  const unsigned char *s = (const unsigned char *) src;
  while (n-- != 0u)
    *d++ = *s++;
  return dest;
}

void *__attribute__ ((weak))
memmove (void *dest, const void *src, size_t n)
{
  unsigned char *d = (unsigned char *) dest;
  const unsigned char *s = (const unsigned char *) src;
  if (d < s)
    {
      while (n-- != 0u)
        *d++ = *s++;
    }
  else if (d > s)
    {
      d += n;
      s += n;
      while (n-- != 0u)
        *--d = *--s;
    }
  return dest;
}

void *__attribute__ ((weak))
memset (void *dest, int c, size_t n)
{
  unsigned char *d = (unsigned char *) dest;
  while (n-- != 0u)
    *d++ = (unsigned char) c;
  return dest;
}

int __attribute__ ((weak))
memcmp (const void *a, const void *b, size_t n)
{
  const unsigned char *pa = (const unsigned char *) a;
  const unsigned char *pb = (const unsigned char *) b;
  while (n-- != 0u)
    {
      if (*pa != *pb)
        return (int) *pa - (int) *pb;
      pa++;
      pb++;
    }
  return 0;
}

size_t __attribute__ ((weak))
strlen (const char *s)
{
  size_t n = 0;
  while (s[n] != '\0')
    n++;
  return n;
}

int __attribute__ ((weak))
strcmp (const char *a, const char *b)
{
  while (*a != '\0' && *a == *b)
    {
      a++;
      b++;
    }
  return (int) (unsigned char) *a - (int) (unsigned char) *b;
}

int __attribute__ ((weak))
strncmp (const char *a, const char *b, size_t n)
{
  while (n != 0u && *a != '\0' && *a == *b)
    {
      a++;
      b++;
      n--;
    }
  if (n == 0u)
    return 0;
  return (int) (unsigned char) *a - (int) (unsigned char) *b;
}

char *__attribute__ ((weak))
strcpy (char *dest, const char *src)
{
  char *d = dest;
  while ((*d++ = *src++) != '\0')
    ;
  return dest;
}

char *__attribute__ ((weak))
strchr (const char *s, int c)
{
  char ch = (char) c;
  for (;; s++)
    {
      if (*s == ch)
        return (char *) s;
      if (*s == '\0')
        return (void *) 0;
    }
}

/* newlib-compatible ctype classification table. The toolchain's <ctype.h>
 * macros index (_ctype_ + 1)[c]; with -nostdlib nothing supplies the table,
 * so the port does. Bit encoding is newlib's: _U 01 upper, _L 02 lower,
 * _N 04 digit, _S 010 space, _P 020 punct, _C 040 control, _X 0100 hex
 * letter, _B 0200 blank. ASCII only; rows 128..255 are zero.
 */
#define UMBRA_CT_U 001
#define UMBRA_CT_L 002
#define UMBRA_CT_N 004
#define UMBRA_CT_S 010
#define UMBRA_CT_P 020
#define UMBRA_CT_C 040
#define UMBRA_CT_X 0100
#define UMBRA_CT_B 0200

#define UMBRA_CT_ROW16(v) v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v

const char __attribute__ ((weak)) _ctype_[1 + 256] = {
  0,                                                        /* EOF slot */
  UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C,           /* 00-03 */
  UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C,           /* 04-07 */
  UMBRA_CT_C,                                               /* BS */
  UMBRA_CT_C | UMBRA_CT_S,                                  /* TAB */
  UMBRA_CT_C | UMBRA_CT_S, UMBRA_CT_C | UMBRA_CT_S,         /* LF VT */
  UMBRA_CT_C | UMBRA_CT_S, UMBRA_CT_C | UMBRA_CT_S,         /* FF CR */
  UMBRA_CT_C, UMBRA_CT_C,                                   /* 0E-0F */
  UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C,           /* 10-13 */
  UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C,           /* 14-17 */
  UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C,           /* 18-1B */
  UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C, UMBRA_CT_C,           /* 1C-1F */
  UMBRA_CT_S | UMBRA_CT_B,                                  /* space */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,           /* !"#$ */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,           /* %&'( */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,           /* )*+, */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,                       /* -./ */
  UMBRA_CT_N, UMBRA_CT_N, UMBRA_CT_N, UMBRA_CT_N,           /* 0123 */
  UMBRA_CT_N, UMBRA_CT_N, UMBRA_CT_N, UMBRA_CT_N,           /* 4567 */
  UMBRA_CT_N, UMBRA_CT_N,                                   /* 89 */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,           /* :;<= */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,                       /* >?@ */
  UMBRA_CT_U | UMBRA_CT_X, UMBRA_CT_U | UMBRA_CT_X,         /* AB */
  UMBRA_CT_U | UMBRA_CT_X, UMBRA_CT_U | UMBRA_CT_X,         /* CD */
  UMBRA_CT_U | UMBRA_CT_X, UMBRA_CT_U | UMBRA_CT_X,         /* EF */
  UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U,           /* GHIJ */
  UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U,           /* KLMN */
  UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U,           /* OPQR */
  UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U,           /* STUV */
  UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U, UMBRA_CT_U,           /* WXYZ */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,           /* [\]^ */
  UMBRA_CT_P, UMBRA_CT_P,                                   /* _` */
  UMBRA_CT_L | UMBRA_CT_X, UMBRA_CT_L | UMBRA_CT_X,         /* ab */
  UMBRA_CT_L | UMBRA_CT_X, UMBRA_CT_L | UMBRA_CT_X,         /* cd */
  UMBRA_CT_L | UMBRA_CT_X, UMBRA_CT_L | UMBRA_CT_X,         /* ef */
  UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L,           /* ghij */
  UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L,           /* klmn */
  UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L,           /* opqr */
  UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L,           /* stuv */
  UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L, UMBRA_CT_L,           /* wxyz */
  UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P, UMBRA_CT_P,           /* {|}~ */
  UMBRA_CT_C,                                               /* DEL */
  UMBRA_CT_ROW16 (0), UMBRA_CT_ROW16 (0),                   /* 80-9F */
  UMBRA_CT_ROW16 (0), UMBRA_CT_ROW16 (0),                   /* -BF */
  UMBRA_CT_ROW16 (0), UMBRA_CT_ROW16 (0),                   /* -DF */
  UMBRA_CT_ROW16 (0), UMBRA_CT_ROW16 (0)                    /* E0-FF */
};

/* newlib's libm reports domain errors through __errno; with -nostdlib the
 * reentrancy machinery is absent, so the port supplies one static cell.
 */
static int umbra_errno_value;

int *__attribute__ ((weak))
__errno (void)
{
  return &umbra_errno_value;
}
