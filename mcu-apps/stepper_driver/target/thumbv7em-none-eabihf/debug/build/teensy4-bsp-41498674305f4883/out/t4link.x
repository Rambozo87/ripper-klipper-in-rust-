/* Memory map for 'Imxrt1060' with custom flash length 2031616. */
MEMORY {
FLASH (RX) : ORIGIN = 0x60000000, LENGTH = 0x1F0000
ITCM (RWX) : ORIGIN = 0x0, LENGTH = 0x30000
DTCM (RWX) : ORIGIN = 0x20000000, LENGTH = 0x50000
OCRAM (RWX) : ORIGIN = 0x20200000, LENGTH = 0x80000
}
__fcb_offset = 0x0;
/* ===--- Begin imxrt-boot-header.x ---===
 * This extra content is injected into the linker script depending on the
 * runtime configuration.
 */

/* If you're ever playing with the boot ROM copy, this is your image size.
 *
 * Note that it depends on the section layout! Need to represent contiguous
 * sections starting from the boot header.
 */
__image_size = SIZEOF(.boot) + SIZEOF(.vector_table) + SIZEOF(.xip) + SIZEOF(.text) + SIZEOF(.rodata);

/* END TODO */
EXTERN(FLEXSPI_CONFIGURATION_BLOCK);

/* # Sections */
SECTIONS
{
  /* Boot header for serial NOR FlexSPI XIP.
   *
   * It's 'XIP' in that it starts executing instructions
   * from flash immediately out of reset. The runtime then
   * manually copies instructions (data, etc.), and we jump
   * to that. After that jump, we're no longer XIP.
   *
   * The i.MX RT boot ROM also supports a way to copy the
   * application image by changing the boot data configuration.
   * Specifically, point the 'start of image' to somewhere other
   * than the start of flash, and specify how many bytes to copy.
   * The boot ROM copies the image, then jumps to the vector table.
   * There's a catch: the boot ROM copies the first 8K from the
   * start of flash too. This represents the entire boot header,
   * including the FCB, IVT, and boot data. (NXP docs say that the
   * initial load region is 4K; my testing shows that it's 8K, and
   * this aligns with observations of others.) If you ever want to
   * try this, make sure you're specifing the VMA and LMA of the
   * boot head section to represent this 8K relocation.
   */
  .boot ORIGIN(FLASH):
  {
    . += __fcb_offset;          /* Changes based on the chip */
    KEEP(*(.fcb));
    . = ORIGIN(FLASH) + 0x1000;
    /* ------------------
     * Image vector table
     * ------------------
     *
     * Not to be confused with the ARM vector table. This tells the boot ROM
     * where to find the boot data and (eventual) first vector table.
     * The IVT needs to reside right here.
     */
    __ivt = .;
    LONG(0x402000D1);           /* Header, magic number */
    LONG(__sivector_table);     /* Address of the vectors table */
    LONG(0x00000000);           /* RESERVED */
    LONG(__dcd);                /* Device Configuration Data */
    LONG(__boot_data);          /* Address to boot data */
    LONG(__ivt);                /* Self reference */
    LONG(0x00000000);           /* Command Sequence File (unused) */
    LONG(0x00000000);           /* RESERVED */
    /* ---------
      * Boot data
      * ---------
      */
    __boot_data = .;
    LONG(ORIGIN(FLASH));        /* Start of image */
    LONG(__image_size);         /* Length of image */
    LONG(0x00000000);           /* Plugin flag (unused) */
    LONG(0xDEADBEEF);           /* Dummy to align boot data to 16 bytes */
    . = ALIGN(4);
    __dcd_start = .;
    KEEP(*(.dcd));              /* Device Configuration Data */
    __dcd_end = .;
    __dcd = ((__dcd_end - __dcd_start) > 0) ? __dcd_start : ABSOLUTE(0);
    *(.Reset);                  /* Jam the imxrt-rt reset handler into flash. */
    *(.__pre_init);             /* Also jam the pre-init function, since we need it to run before instructions are placed. */
    . = ORIGIN(FLASH) + 0x2000;   /* Reserve the remaining 8K as a convenience for a non-XIP boot. */
  } > FLASH
}

ASSERT((__dcd_end - __dcd_start) % 4 == 0, "
ERROR(imxrt-rt): .dcd (Device Configuration Data) size must be a multiple of 4 bytes.");

/* ===--- End imxrt-boot-header.x ---=== */
INCLUDE device.x
REGION_ALIAS("REGION_TEXT", ITCM);
REGION_ALIAS("REGION_VTABLE", DTCM);
REGION_ALIAS("REGION_RODATA", OCRAM);
REGION_ALIAS("REGION_DATA", DTCM);
REGION_ALIAS("REGION_BSS", DTCM);
REGION_ALIAS("REGION_UNINIT", OCRAM);
REGION_ALIAS("REGION_STACK", DTCM);
REGION_ALIAS("REGION_HEAP", OCRAM);
__stack_size = 0x00004000;
__heap_size = 0x00004000;
REGION_ALIAS("REGION_LOAD_VTABLE", FLASH);
REGION_ALIAS("REGION_LOAD_TEXT", FLASH);
REGION_ALIAS("REGION_LOAD_RODATA", FLASH);
REGION_ALIAS("REGION_LOAD_DATA", FLASH);
__flexram_config = 0xFFFAAAAA;
__imxrt_family = 1060;
/* ===--- Begin imxrt-link.x ---===
 * This section of the linker script is a fork of the default linker script provided by
 * imxrt-rt, version 0.7.1. It's modified to support the needs of imxrt-rt.
 */

/* # Entry point = reset vector */
EXTERN(__RESET_VECTOR);
EXTERN(Reset);
ENTRY(Reset);

/* # Exception vectors */
/* This is effectively weak aliasing at the linker level */
/* The user can override any of these aliases by defining the corresponding symbol themselves (cf.
   the `exception!` macro) */
EXTERN(__EXCEPTIONS); /* depends on all the these PROVIDED symbols */

EXTERN(DefaultHandler);
EXTERN(__pre_init);

PROVIDE(NonMaskableInt = DefaultHandler);
EXTERN(HardFaultTrampoline);
PROVIDE(MemoryManagement = DefaultHandler);
PROVIDE(BusFault = DefaultHandler);
PROVIDE(UsageFault = DefaultHandler);
PROVIDE(SecureFault = DefaultHandler);
PROVIDE(SVCall = DefaultHandler);
PROVIDE(DebugMonitor = DefaultHandler);
PROVIDE(PendSV = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);

PROVIDE(DefaultHandler = DefaultHandler_);
PROVIDE(HardFault = HardFault_);

/* # Interrupt vectors */
EXTERN(__INTERRUPTS); /* `static` variable similar to `__EXCEPTIONS` */

/* # Sections */
SECTIONS
{
  .stack (NOLOAD) : ALIGN(8)
  {
    __estack = .;
    . += ALIGN(__stack_size, 8);
    __sstack = .;
    /* Symbol expected by cortex-m-rt */
    _stack_start = __sstack;
  } > REGION_STACK

  .vector_table : ALIGN(1024)
  {
    FILL(0xff);
    __vector_table = .;
    __svector_table = .;

    /* Initial Stack Pointer (SP) value */
    LONG(__sstack);

    /* Reset vector */
    KEEP(*(.vector_table.reset_vector)); /* this is the `__RESET_VECTOR` symbol */
    __reset_vector = .;

    /* Exceptions */
    KEEP(*(.vector_table.exceptions)); /* this is the `__EXCEPTIONS` symbol */
    __eexceptions = .;

    /* Device specific interrupts */
    KEEP(*(.vector_table.interrupts)); /* this is the `__INTERRUPTS` symbol */
    __evector_table = .;
  } > REGION_VTABLE AT> REGION_LOAD_VTABLE
  __sivector_table = LOADADDR(.vector_table);

  /* This section guarantees VMA = LMA to allow the execute-in-place entry point to be inside the image. */
  .xip : ALIGN(4)
  {
    /* Included here if not otherwise included in the boot header. */
    *(.Reset);
    *(.__pre_init);
    *(.xip .xip.*);
  } > REGION_LOAD_TEXT

  .text : ALIGN(4)
  {
    FILL(0xff);
    __stext = .;
    *(.text .text.*);
    /* The HardFaultTrampoline uses the `b` instruction to enter `HardFault`,
       so must be placed close to it. */
    *(.HardFaultTrampoline);
    *(.HardFault.*);
    . = ALIGN(4); /* Pad .text to the alignment to workaround overlapping load section bug in old lld */
    __etext = .;
  } > REGION_TEXT AT> REGION_LOAD_TEXT
  __sitext = LOADADDR(.text);

  .rodata : ALIGN(4)
  {
    FILL(0xff);
    . = ALIGN(4);
    __srodata = .;
    *(.rodata .rodata.*);

    /* 4-byte align the end (VMA) of this section.
       This is required by LLD to ensure the LMA of the following .data
       section will have the correct alignment. */
    . = ALIGN(4);
    __erodata = .;
  } > REGION_RODATA AT> REGION_LOAD_RODATA
  __sirodata = LOADADDR(.rodata);

  .data : ALIGN(4)
  {
    FILL(0xff);
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
    __edata = .;
  } > REGION_DATA AT> REGION_LOAD_DATA
  __sidata = LOADADDR(.data);

  .bss (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __sbss = .;
    *(.bss .bss.*);
    *(COMMON); /* Uninitialized C statics */
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
      __ebss = .;
  } > REGION_BSS

  .uninit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __suninit = .;
    *(.uninit .uninit.*);
    . = ALIGN(4);
    __euninit = .;
  } > REGION_UNINIT

  .heap (NOLOAD) : ALIGN(4)
  {
    __sheap = .;
    . += ALIGN(__heap_size, 4);
    __eheap = .;
  } > REGION_HEAP

  /* Dynamic relocations are unsupported. This section is only used to detect relocatable code in
     the input files and raise an error if relocatable code is found */
  .got (NOLOAD) :
  {
    KEEP(*(.got .got.*));
  }

  /DISCARD/ :
  {
    /* Unused exception related info that only wastes space */
    *(.ARM.exidx);
    *(.ARM.exidx.*);
    *(.ARM.extab.*);
  }
}

/* Do not exceed this mark in the error messages below                                    | */
/* # Alignment checks */

ASSERT(__sstack % 8 == 0 && __estack % 8 == 0, "
BUG(imxrt-rt): .stack is not 8-byte aligned");

ASSERT(__sdata % 4 == 0 && __edata % 4 == 0, "
BUG(imxrt-rt): .data is not 4-byte aligned");

ASSERT(__sidata % 4 == 0, "
BUG(imxrt-rt): the LMA of .data is not 4-byte aligned");

ASSERT(__sbss % 4 == 0 && __ebss % 4 == 0, "
BUG(imxrt-rt): .bss is not 4-byte aligned");

ASSERT(__sheap % 4 == 0, "
BUG(imxrt-rt): start of .heap is not 4-byte aligned");

/* # Position checks */

/* ## .vector_table */
ASSERT(__reset_vector == ADDR(.vector_table) + 0x8, "
BUG(imxrt-rt): the reset vector is missing");

ASSERT(__eexceptions == ADDR(.vector_table) + 0x40, "
BUG(imxrt-rt): the exception vectors are missing");

ASSERT(SIZEOF(.vector_table) > 0x40, "
ERROR(imxrt-rt): The interrupt vectors are missing.
Possible solutions, from most likely to less likely:
- Link to imxrt-ral, or another compatible device crate
- Check that you actually use the device/hal/bsp crate in your code
- Disable the 'device' feature of cortex-m-rt to build a generic application (a dependency
may be enabling it)
- Supply the interrupt handlers yourself. Check the documentation for details.");

/* # Other checks */
ASSERT(SIZEOF(.got) == 0, "
ERROR(imxrt-rt): .got section detected in the input object files
Dynamic relocations are not supported. If you are linking to C code compiled using
the 'cc' crate then modify your build script to compile the C code _without_
the -fPIC flag. See the documentation of the `cc::Build.pic` method for details.");

/* Do not exceed this mark in the error messages above                                    | */

/* ===--- End imxrt-link.x ---=== */
