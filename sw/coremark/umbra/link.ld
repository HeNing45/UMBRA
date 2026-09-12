OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 0x0003e000
}

PHDRS
{
  text PT_LOAD FLAGS(5);
  data PT_LOAD FLAGS(6);
}

SECTIONS
{
  . = ORIGIN(RAM);

  .text : ALIGN(4)
  {
    KEEP(*(.text.init))
    *(.text .text.*)
  } > RAM :text

  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*)
  } > RAM :text

  /*
 * Profile-guided optimization support. With -fprofile-info-section the
 * compiler emits one pointer per translation unit here; libgcov's
 * __gcov_info_to_gcda walks the range at run time. Empty and inert in every
 * build that does not pass that flag, so the pinned -O2 gate image is
 * unaffected.
 */
  .gcov_info : ALIGN(4)
  {
    PROVIDE(__gcov_info_start = .);
    KEEP(*(.gcov_info))
    PROVIDE(__gcov_info_end = .);
  } > RAM :data

  .data : ALIGN(4)
  {
    *(.data .data.*)
  } > RAM :data

  .sdata : ALIGN(4)
  {
    __global_pointer$ = . + 0x800;
    *(.sdata .sdata.*)
  } > RAM :data

  .bss (NOLOAD) : ALIGN(4)
  {
    __bss_start = .;
    *(.sbss .sbss.*)
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end = .;
  } > RAM :data

  __image_end = .;
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);

  /DISCARD/ :
  {
    *(.comment)
    *(.note .note.*)
  }

  ASSERT(__image_end <= (__stack_top - 0x4000),
         "CoreMark image leaves less than 16 KiB for stack")
}
