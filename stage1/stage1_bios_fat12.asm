;
;
;   Small OS template
;   Boot Loader (SMALLOS)
;
;   stage1_bios_fat12.asm
;   BIOS Boot loader for FAT12
;
;   Copyright(C) 2012,2013,2018 Tsukasa Oi.
;   Copyright(C) 2012 Hideki EIRAKU.
;
;
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions
;   are met:
;
;   *  Redistributions of source code must retain the above copyright
;      notice, this list of conditions and the following disclaimer.
;   *  Redistributions in binary form must reproduce the above
;      copyright notice, this list of conditions and the following
;      disclaimer in the documentation and/or other materials provided
;      with the distribution.
;   *  The names of its contributors may not be used to endorse or
;      promote products derived from this software without specific
;      prior written permission.
;
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
;   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
;   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
;   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
;   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;
;
cpu  8086
bits 16
org  0x7c00

;; Implementation Notes:
;; * This boot loader is segment-independent while it is loaded on
;;   linear address 0x07c00. Location may be 0000:7c00 or 07c0:0000.
;; * It includes self-modification of the boot loader itself.

_start:
	jmp short start
	nop

;; Machine Assumptions:
;; * 8086 with/without MOV SS errata or later
;; * 32KiB memory or more
;; * Known PC/AT BIOS
;; BIOS Parameter Block (BPB) / File System Assumptions:
;; * FAT extended BPB
;; * FAT12 with no extensions
;; * 12-bit clusters
;; * 32-bit LBA (max: 2TB in 512-bytes sector disk)
;; * 512/1024/2048/4096-bytes sector size (according to the FAT spec)
;;   (must match sector size identified by BIOS)

;; Sample BPB:
;; based on mkdosfs BPB from 1.44MB 2HD floppy disk image
_bpb_oem_name:
	db "SMALLOS"
	times 8-($-_bpb_oem_name) db " "
_bpb_numbytes_sec:
	dw 0x0200
_bpb_numsec_clus:
	db 0x01
_bpb_numsec_rsvd:
	dw 0x0001
_bpb_numfat:
	db 0x02
_bpb_numroot:
	dw 0x00e0
_bpb_numsec_total16:
	dw 0x0b40
_bpb_media_descriptor:
	db 0xf0
_bpb_numsec_fat16:
	dw 0x0009
_bpb_numsec_track:
	dw 0x0012
_bpb_numheads:
	dw 0x0002
_bpb_numsec_hidden:
	dd 0x00000000
_bpb_numsec_total32:
	dd 0x00000000
_bpb_physical_drive:
	db 0x00
_bpb_fat16_reserved:
	db 0x00
_bpb_ext_boot_sign:
	db 0x29
_bpb_volume_id:
	dd 0x12345678
_bpb_volume_label:
	db "SMALLOS"
	times 11-($-_bpb_volume_label) db " "
_bpb_fs_type:
	db "FAT12"
	times 8-($-_bpb_fs_type) db " "

;; Addresses used for loading (4k-aligned)
%define ADRH_INIT   0x10
%define ADRH_LIMIT  0x50
%define ADRH_INFO   0x50
%define ADDR_INIT   0x100 * ADRH_INIT
%define ADDR_LIMIT  0x100 * ADRH_LIMIT
%define ADDR_INFO   0x100 * ADRH_INFO
%define SEG_INIT    0x10 * ADRH_INIT

;; Addresses for stack and temporal variables
%define STK_BASE        0x7c00
%define tmp_dx          0x7bfe
%define tmp_tstate      0x7bfc
%define tmp_numroot     0x7bfa
%define tmp_offsec_fat  0x7bf6
%define tmp_offsec_data 0x7bf2

;; Addresses for optimization
;; bpb_*    : help NASM to optimize fixed addresses
;; REL_BASE : help optimizing addressing (by using fixed base address)
%define bpb_oem_name            0x7c03
%define bpb_numbytes_sec        0x7c0b
%define bpb_numsec_clus         0x7c0d
%define bpb_numsec_rsvd         0x7c0e
%define bpb_numfat              0x7c10
%define bpb_numroot             0x7c11
%define bpb_numsec_total16      0x7c13
%define bpb_media_descriptor    0x7c15
%define bpb_numsec_fat16        0x7c16
%define bpb_numsec_track        0x7c18
%define bpb_numheads            0x7c1a
%define bpb_numsec_hidden       0x7c1c
%define bpb_numsec_total32      0x7c20
%define bpb_physical_drive      0x7c24
%define bpb_fat16_reserved      0x7c25
%define bpb_ext_boot_sign       0x7c26
%define bpb_volume_id           0x7c27
%define bpb_volume_label        0x7c2b
%define bpb_fs_type             0x7c36
%define REL_BASE                bpb_numbytes_sec

;; Relative addressing macros
%define rel(x) di-REL_BASE+x
%define rel_bpb(x) rel(bpb_%+x)
%define rel_tmp(x) rel(tmp_%+x)



start:
	; DL == drive
	; IF := 0
	cli
	; AX := 0
	; DS := 0
	; ES := 0
	; SS := 0
	xor  ax, ax
	mov  ds, ax
	mov  es, ax
	mov  ss, ax
	; SP := STK_BASE
	; DI := REL_BASE
	mov  sp, STK_BASE
	mov  di, REL_BASE
	push dx                     ; tmp_dx
	push ax                     ; tmp_tstate := { 0, 0 }
	push word[rel_bpb(numroot)] ; tmp_numroot

disk_reset:
	; BIOS CALL: RESET DISK
	; AH == 0
	; DL == drive
	; CF := error
	int  0x13
	jc   short _err_boot_1p

disk_check_ext13h:
	mov  ah, 0x41
	mov  bx, 0x55aa
	; BIOS CALL: CHECK EXTENDED INT13H
	; AH == 0x41
	; BX == 0x55AA
	; if EXTENDED INT13H is supported:
	;    CF := 0
	;    BX := 0xAA55
	;    CX := (other flags) | 1
	int  0x13
	jc   short .noext
	sub  bx, 0xaa55
	jne  short .noext
	shr  cx, 1
	jnc  short .noext
	.ext:
	dec  byte[rel_tmp(tstate)+1] ; tmp_tstate[1] := 0xff
	.noext:

fs_calc_geom:
	xor  ax, ax
	; SI:BP := dword(numsec_hidden) + word(numsec_rsvd)
	;       == dword(offsec_fat)
	mov  bp, [rel_bpb(numsec_hidden)]
	mov  si, [rel_bpb(numsec_hidden)+2]
	add  bp, [rel_bpb(numsec_rsvd)]
	adc  si, ax
	push si ; tmp_offsec_fat+2
	push bp ; tmp_offsec_fat
	; byte(numfat) := 0
	; SI:BP        := dword(offsec_root)
	.calcroot:
		add  bp, [rel_bpb(numsec_fat16)]
		adc  si, ax
		dec  byte[rel_bpb(numfat)]
		jnz  short .calcroot
	; AH == 0
	; AX := 32
	mov  al, 32
	mul  word[rel_bpb(numroot)]
	; DX:AX == 32 * word(numroot)
	; word(numbytes_sec) > 32 (according to FAT spec)
	; AX := (32 * word(numroot)) / word(numbytes_sec)
	; DX := (32 * word(numroot)) % word(numbytes_sec)
	div  word[rel_bpb(numbytes_sec)]
	; CX := (32 * word(numroot) + word(numbytes_sec) - 1) / word(numbytes_sec)
	;    == sectors of root directory entries
	; 0 < CX < 4096 (according to FAT spec)
	; 0 <= DX < 4096 (according to FAT spec)
	; thus sign flag after computation of (DX-1) is 1 if DX == 0
	dec  dx
	js   short .norem
	.rem:
	inc  ax
	.norem:
	xchg ax, cx
	; DX:AX := SI:BP == dword(offsec_root)
	xchg ax, bp
	mov  dx, si
	; BX := ADDR_INIT
	mov  bx, ADDR_INIT

;; This loop is performed for two different conditions:
;; - While finding boot file
;;     - BX == ADDR_INIT
;;     - CX == remaining root directory entry sectors
;;     - err_boot if CX == 0
;; - While reading boot file
;;     - BX >= ADDR_INIT
;;     - CX == remaining sectors in cluster
;;     - Traverse FAT if CX == 0
disk_read_loop:
	call read_disk
	rol  byte[rel_tmp(tstate)+0], 1
	jc   short boot_find_file.end
	boot_find_file:
		; BL == 0 (according to the FAT spec [no 128-byte sector or some])
		; BP := BX
		;    == ADDR_INIT + word(numbytes_sec)
		; SI := BX
		;    == ADDR_INIT
		mov  bp, bx
		mov  bh, ADRH_INIT
		mov  si, bx
		.findloop:
			; BL == 0 (according to FAT spec and 4K or sector aligned BX values)
			; SI == directory entry
			; +00-0A: FILE NAME
			; +0B-0B: FILE ATTRIBUTES
			; +0C-0C: RESERVED
			; +0D-0D: CREATE_TIME_MS
			; +0E-0F: CREATE_TIME
			; +10-11: CREATE_DATE
			; +12-13: ACCESS_DATE
			; +14-15: CLUSTER_HI
			; +16-17: MODIFY_TIME
			; +18-19: MODIFY_DATE
			; +1A-1B: CLUSTER_LO
			; +1C-1F: FILE_SIZE
			; [SI+0]    == file name (first byte)
			;              no subsequent entries if 0
			; [SI+0x0B] == file attributes
			; File attributes to reject:
			;  0x08: Volume Label (or VFAT File Name)
			;  0x10: Subdirectory
			;  0x40: Device
			cmp  byte[si], bl
			je   short _err_boot_1p
			test byte[si+0x0b], 0x58
			jnz  short .next
			; CX == directory table loop remaining (in sectors)
			; CX := 11 (length of file name)
			; SI == directory entry
			; DI := boot_file
			; DF := 0
			push cx
			push si
			push di
			mov  cx, 0x000b
			mov  di, boot_file
			cld
			repe cmpsb
			pop  di
			pop  si
			pop  cx
			; found directory entry if file names are equal
			je   short boot_found_file
			.next:
			dec  word [rel_tmp(numroot)]
			jz   short _err_boot_1p
			; SI := next directory entry
			add  si, byte 0x20
			cmp  si, bp
			jb   short .findloop
		.end:
	; BL == 0
	; boot if BX >= ADDR_LIMIT
	cmp  bh, ADRH_LIMIT
	jae  short boot
	loop disk_read_loop

_err_boot_1p:
	rol  byte[rel_tmp(tstate)+0], 1
	jnc  short err_boot

fs_traverse_fat:
	pop  ax
	; CX    == 0
	; CX    := 3
	; AX    == word(cluster)
	; DX:AX := word(cluster) * 3
	mov  cl, 3
	mul  cx
	push ax
	; AX <= (4096-8) * 3 / 2
	; DX == 0
	; DX:AX := word(cluster) * 3 / 2
	shr  ax, 1
	; AX := word(offsec_clus_fat)
	; DX := word(offbytes_clus_fat)
	div  word[rel_bpb(numbytes_sec)]
	push bx
	push dx
	; AX  < 12
	; DX := 0
	cwd
	; DX:AX := dword(sec_clus_fat)
	; read 2 sectors because FAT information may exceed sector-boundary.
	add  ax, [rel_tmp(offsec_fat)]
	adc  dx, [rel_tmp(offsec_fat)+2]
	mov  bh, ADRH_INFO
	call read_disk
	call read_disk
	; AX := word(cluster)
	pop  si
	pop  bx
	pop  bp
	mov  ax, [si+ADDR_INFO]
	shr  bp, 1
	jnc  short .even
	.odd:
	mov  cl, 4
	shr  ax, cl
	.even:
	and  ax, 0x0fff

fs_traverse_fat_next:
	; AX == word(cluster)
	; error if AX < 2 or AX >= 0x0ff8
	; AX := word(cluster) - 2
	push ax
	cmp  ax, 0x0ff7
	ja   short boot
	sub  ax, 2
	jb   short err_boot
	; AX    == word(cluster) - 2
	; DX:AX := dword(offsec_data) + byte(numsec_clus) * (word(cluster) - 2)
	mov  cl, [rel_bpb(numsec_clus)]
	mul  cx
	add  ax, [rel_tmp(offsec_data)]
	adc  dx, [rel_tmp(offsec_data)+2]
	jmp  near disk_read_loop



; FINAL PROCEDURE
; CONTEXT
;  jump from FAT-related functions
boot:
	; DL := drive
	; DH := file system type
	; jump to ADDR_INIT
	mov  dl, [rel_tmp(dx)]
	mov  dh, 4
	jmp  SEG_INIT:0



; BOOT PROCEDURE
; CONTEXT (boot_found_file)
;  jump from boot_find_file
; REGISTERS MODIFIED
;  AX:SI == dword(file_clus)
; dword(offsec_data) := DX:AX
;                    := dword(offsec_root) + number of root entry sectors already read
; CX    == number of root entry sectors remaining (not read) + 1
; CX    := 0
_boot_found_file:
	.calcdata:
		inc  ax
		jnz  short .noovf
		.ovf:
		inc  dx
		.noovf:
		boot_found_file:
			loop _boot_found_file.calcdata
	push dx ; tmp_offsec_data+2
	push ax ; tmp_offsec_data
	; SI == directory entry
	; AX := word(file_clus)
	mov  ax, [si+0x1a]
	dec  byte[rel_tmp(tstate)+0] ; tmp_tstate[0] := 0xff
	; Back to FAT traversal
	jmp  short fs_traverse_fat_next



; TERMINATOR PROCEDURE
err_boot:
	mov  si, boot_error_msg
	mov  cx, boot_error_msg_end-boot_error_msg
	.charloop:
		; SI == current char pointer
		; SI++
		; DF := 0
		; AL := char to print
		cld
		lodsb
		mov  ah, 0x0e
		mov  bx, 0x0007
		; BIOS CALL: PRINT A CHARACTER
		; AH == 0x0E
		; AL == char to print
		; BH == page
		; BL == colors
		int  0x10
		loop .charloop
	; CX == 0
	; AX := 0
	xchg ax, cx
	; BIOS CALL: WAIT FOR KEY
	; AH == 0
	int  0x16
	; BIOS CALL: REBOOT
	int  0x19
	; just padding :)
	.final:
		hlt
		jmp  short .final



; DISK PROCEDURE
; CONTEXT
;  call from fs_traverse_fat (twice)
; PARAMETERS
;  DX:AX == LBA
;  BX    == BUFFER
; REGISTERS MODIFIED
;  DX:AX := LBA+1
;  BX    := BUFFER+numbytes_sec
; REGISTERS PRESERVED
;  CX, DI
; REGISTERS DESTROYED
;  BP, SI
read_disk:
	push ax
	push cx
	push dx
	rol  byte[rel_tmp(tstate)+1], 1
	jnc  short .noext
	.ext:
		; AX == LBA_LO
		; DX == LBA_HI
		; BX == BUFFER
		; EXTENDED READ DISK
		; AH := 0x42
		; SI := SP (pushed to the stack on demand)
		; Contents of ext13h buffer(si):
		; +00: 0x0010 (size of ext13h_buf)
		; +02: 0x0001 (sectors to read)
		; +04: BUFFER
		; +06: 0x0000 (segment 0)
		; +08: LBA_LO
		; +0a: LBA_HI
		; +0c: 0x0000
		; +0e: 0x0000
		xor  cx, cx
		push cx
		push cx
		push dx
		push ax
		push cx
		push bx
		mov  cl, 0x01
		push cx
		mov  cl, 0x10
		push cx
		mov  si, sp
		mov  ah, 0x42
		jmp  short .perform
	.noext:
		sub  sp, byte 0x10
		; DX:AX == dword(LBA)
		; BP    := word(numsec_track)
		; CX:AX := dword(LBA)
		mov  bp, [rel_bpb(numsec_track)]
		mov  cx, dx
		; word(numsec_track) <= 63 (according to disk geometry identified by BIOS)
		; CX:AX := dword(LBA) / word(numsec_track)
		;       == dword(HEAD_CYLINDER)
		; DX    := dword(LBA) % word(numsec_track)
		call div32
		; SI := DX == word(SECTOR)
		inc  dx
		mov  si, dx
		; BP    := word(numheads)
		; CX:AX := dword(HEAD_CYLINDER) / word(numheads)
		;       == dword(CYLINDER)
		; DX    := dword(HEAD_CYLINDER) % word(numheads)
		;       == word(HEAD)
		mov  bp, [rel_bpb(numheads)]
		call div32
		; [INT 13h's CYLINDER is a word value (discard CX)]
		; CL := 6
		; AL == byte(CYLINDER_LO)
		; AH := byte(CYLINDER_HI) << 6
		mov  cl, 6
		shl  ah, cl
		; AL := byte(CYLINDER_HI) << 6 | byte(SECTOR)
		; AH := byte(CYLINDER_LO)
		xchg al, ah
		or   ax, si
		; AX == word(CYLINDER/SECTOR)
		; CX := word(CYLINDER/SECTOR)
		xchg ax, cx
		; DL == byte(HEAD)
		; DH := byte(HEAD)
		mov  dh, dl
		; READ DISK
		; AX := 0x0201 (0x02: read, 0x01: sectors to read)
		; DH == byte(HEAD)
		; CX == word(CYLINDER/SECTOR)
		; BX == BUFFER
		mov  ax, 0x0201
	.perform:
		; DL := byte(drive)
		mov  dl, [rel_tmp(dx)]
		int  0x13
		sti
		; CF == error
		jc   short err_boot
		add  sp, byte 0x10
		pop  dx
		pop  cx
		pop  ax
		; BX += word(numbytes_sec)
		; DX:AX++
		add  bx, [rel_bpb(numbytes_sec)]
		inc  ax
		jnz  short .noovf
	.ovf:
		inc  dx
	.noovf:
		ret



; UTILITY PROCEDURE (u32 / u16)
; PARAMETERS
;  CX:AX: DIVIDEND
;  BP:    DIVISOR
; REGISTERS MODIFIED
;  CX:AX: DIVIDEND / DIVISOR
;  DX:    DIVIDEND % DIVISOR
div32:
	xchg ax, cx
	xor  dx, dx
	div  bp
	xchg ax, cx
	div  bp
	ret



boot_file:
	db	'STAGE2  SYS'
boot_error_msg:
	db	'Boot Error.',0x0d,0x0a
	db	'Press Any Key to Restart...',0x0d,0x0a
boot_error_msg_end:



_end:
	times 512-2-($-$$) db 0x00
	dw	0xaa55
