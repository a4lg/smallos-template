;
;
;   Small OS template
;   Boot Loader (SMALLOS)
;
;   stage1_bios_fat32.asm
;   BIOS Boot loader for FAT32
;
;   Copyright(C) 2012,2013 Tsukasa Oi.
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
;; * It destroys some contents of BPB on memory.
;; * It includes self-modification of the boot loader itself.

_start:
	jmp short start
	nop

;; Machine Assumptions:
;; * 8086 with/without MOV SS errata or later
;; * 32KiB memory or more
;; * Known PC/AT BIOS
;; BIOS Parameter Block (BPB) / File System Assumptions:
;; * FAT32 extended BPB
;; * FAT32 with no extension (such as FAT+)
;; * 28-bit clusters
;; * 32-bit LBA (max: 2TB in 512-bytes sector disk)
;; * 512/1024/2048/4096-bytes sector size (according to the FAT spec)
;;   (must match sector size identified by BIOS)

;; Sample BPB:
;; based on mkdosfs BPB from 32MiB raw image
_bpb_oem_name:
	db "SMALLOS"
	times 8-($-_bpb_oem_name) db " "
_bpb_numbytes_sec:
	dw 0x0200
_bpb_numsec_clus:
	db 0x01
_bpb_numsec_rsvd:
	dw 0x0020
_bpb_numfat:
	db 0x02
_bpb_numroot:
	dw 0x0000
_bpb_numsec_total16:
	dw 0x0000
_bpb_media_descriptor:
	db 0xf8
_bpb_numsec_fat16:
	dw 0x0000
_bpb_numsec_track:
	dw 0x0020
_bpb_numheads:
	dw 0x0040
_bpb_numsec_hidden:
	dd 0x00000000
_bpb_numsec_total32:
	dd 0x00010000
_bpb_numsec_fat32:
	dd 0x000001f8
_bpb_mirror_flags:
	dw 0x0000
_bpb_fat32_version:
	db 0, 0
_bpb_offclus_root:
	dd 0x00000002
_bpb_offsec_fsinfo:
	dw 0x0001
_bpb_offsec_backup:
	dw 0x0006
_bpb_fat32_reserved:
	times 12 db 0
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
	db "FAT32"
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
%define tmp_offsec_fat  0x7bfa
%define tmp_offsec_data 0x7bf6

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
%define bpb_numsec_fat32        0x7c24
%define bpb_mirror_flags        0x7c28
%define bpb_fat32_version       0x7c2a
%define bpb_offclus_root        0x7c2c
%define bpb_offsec_fsinfo       0x7c30
%define bpb_offsec_backup       0x7c32
%define bpb_fat32_reserved      0x7c34
%define bpb_physical_drive      0x7c40
%define bpb_fat16_reserved      0x7c41
%define bpb_ext_boot_sign       0x7c42
%define bpb_volume_id           0x7c43
%define bpb_volume_label        0x7c47
%define bpb_fs_type             0x7c52
%define REL_BASE                bpb_numbytes_sec

;; Relative addressing macros
%define rel(x) di-REL_BASE+(x)
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
	push dx ; tmp_dx

disk_reset:
	; BIOS CALL: RESET DISK
	; AH == 0
	; DL == drive
	; CF := error
	int  0x13
	jc   short fs_traverse_loop._err_boot_1cp

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
	; BX == 0
	; BL == 0
	; SMI1 -> SMC1
	mov  byte[read_disk.ext-1], bl
	.noext:

fs_calc_geom:
	; SI:BP := dword(numsec_hidden) + word(numsec_rsvd)
	;       == dword(offsec_fat)
	mov  bp, [rel_bpb(numsec_hidden)]
	mov  si, [rel_bpb(numsec_hidden)+2]
	add  bp, [rel_bpb(numsec_rsvd)]
	adc  si, byte 0
	push si ; tmp_offsec_fat+2
	push bp ; tmp_offsec_fat
	; byte(numfat) := 0
	; SI:BP        := dword(offsec_data)
	.calcdata:
		add  bp, [rel_bpb(numsec_fat32)]
		adc  si, [rel_bpb(numsec_fat32)+2]
		dec  byte[rel_bpb(numfat)]
		jnz  short .calcdata
	push si ; tmp_offsec_data+2
	push bp ; tmp_offsec_data
	; BX := ADDR_INIT
	mov  bx, ADDR_INIT
	; AX:SI := dword(offclus_root)
	mov  ax, [rel_bpb(offclus_root)+2]
	mov  si, [rel_bpb(offclus_root)]

fs_traverse_loop:
	; AX:SI == dword(cluster)
	; AX:SI := dword(cluster) - 2
	; error if AX:SI < 2
	push ax
	push si
	xor  cx, cx
	sub  si, byte 2
	sbb  ax, cx
	._err_boot_1cp:
		jc   short err_boot
	; CX == 0
	; CX := word(numsec_clus)
	mov  cl, [rel_bpb(numsec_clus)]
	; AX == word(cluster-2_hi)
	; AX := word(cluster-2_hi) * word(numsec_clus)
	mul  cx
	; AX := word(cluster-2_lo)
	; SI := word(cluster-2_hi) * word(numsec_clus)
	xchg ax, si
	; DX:AX := word(cluster-2_lo) * word(numsec_clus)
	mul  cx
	; SI    == word(cluster-2_hi) * word(numsec_clus)
	; DX:AX := dword(offsec_clus_read)
	add  dx, si
	add  ax, [rel_tmp(offsec_data)]
	adc  dx, [rel_tmp(offsec_data)+2]
	; CX == word(numsec_clus)
	.sectorloop:
		call read_disk
		; SMC2: e8 xxxx (.boot_find_file) -> be xxxx (MOV si, xxxx)
		jmp short .boot_find_file
		.boot_find_file:
			; BL == 0 (according to the FAT spec [no 128-byte sector or some])
			; BP := BX
			;    == ADDR_INIT + word(numbytes_sec)
			; SI := BX
			;    == ADDR_INIT
			mov  bp, bx
			mov  bh, ADRH_INIT
			mov  si, bx
			.boot_findloop:
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
				je   short err_boot
				test byte[si+0x0b], 0x58
				jnz  short .boot_next
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
				.boot_next:
				; SI := next directory entry
				add  si, byte 0x20
				cmp  si, bp
				jb   short .boot_findloop
		.boot_find_file_end:
		; BL == 0
		; boot if BX >= ADDR_LIMIT
		cmp  bh, ADRH_LIMIT
		jae  short boot
		loop .sectorloop
	; AX:CX := dword(cluster)
	pop  cx
	pop  ax
	push bx
	; AX:CX := dword(cluster) * 4
	add  cx, cx
	adc  ax, ax
	add  cx, cx
	adc  ax, ax
	; BP    := word(numbytes_sec)
	; DX    := word(offbytes_clus_fat)
	; CX:AX := dword(offsec_clus_fat)
	mov  bp, [rel_bpb(numbytes_sec)]
	call div32_short
	; DX:AX := dword(offsec_clus_fat)
	; SI    := word(offbytes_clus_fat)
	; BL    == 0
	; BX    := ADDR_INFO
	xchg dx, cx
	mov  bh, ADRH_INFO
	add  ax, [rel_tmp(offsec_fat)]
	adc  dx, [rel_tmp(offsec_fat)+2]
	call read_disk
	mov  si, cx
	sub  bx, [rel_bpb(numbytes_sec)]
	; [BX+SI] == dword(next_clus)
	; AX:SI   := dword(next_clus)
	; BP      := 0x0FFF
	mov  ax, [bx+si+2]
	mov  si, [bx+si]
	pop  bx
	mov  bp, 0x0fff
	and  ax, bp
	; Normal cluster if AX != 0x0FFF
	cmp  ax, bp
	jne  short fs_traverse_loop
	; Check terminator
	cmp  si, byte -9
	; SMC3: eb xx (err_boot) -> eb xx (boot)
	ja   err_boot
	.to_start:
		jmp  near fs_traverse_loop



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



; FINAL PROCEDURE
; CONTEXT
;  jump from FAT-related functions
boot:
	; DL := drive
	; DH := file system type
	; jump to ADDR_INIT
	mov  dl, [rel_tmp(dx)]
	mov  dh, 6
	jmp  SEG_INIT:0



; BOOT PROCEDURE
; CONTEXT
;  jump from boot_find_file
; REGISTERS MODIFIED
;  AX:SI == dword(file_clus)
boot_found_file:
	; SI    == directory entry
	; AX:SI := dword(file_clus)
	mov  ax, [si+0x14]
	mov  si, [si+0x1a]
	; BL == 0
	; ZF := 1
	; SMI2 -> SMC2
	; SMI3 -> SMC3
	mov  byte[fs_traverse_loop.boot_find_file-1], fs_traverse_loop.boot_find_file_end - fs_traverse_loop.boot_find_file
	mov  byte[fs_traverse_loop.to_start-1], boot - fs_traverse_loop.to_start
	; Back to FAT32 traversal
	jmp  short fs_traverse_loop.to_start



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
	; SMC1: eb xx (_read_disk_noext) -> eb 00
	jmp short .noext
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
div32_short:
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
