OUTPUT_ARCH(arm)
ENTRY(_start)

SECTIONS
{
	. = 0x0;
	.text : { *(.text) }
	. = 0x100;
	.data : { *(.data) }
	PROVIDE (kernel = 0x8000);
}
