Part of Small OS Template
==========================


What is this?
--------------

This is a part of an attempt to write an operating system.
For now, stage 1 BIOS boot loader is available open source
(it seems I accidentally removed all stage2 source files).

Thanks to intensive code golfing, this stage 1 BIOS boot loader
incorporates many features yet very portable.

*	Works on original Intel 8086 with MOV SS errata.
*	Works on 32KiB memory platform.
*	Works on known PC/AT BIOS (all I tested).
	*	Stack consumption on an interrupt must be less than 6KiB.
*	Supports 32-bit LBA with 512/1024/2048/4096-bytes sector size  
	(in other words, all sector sizes as defined in the FAT spec).
