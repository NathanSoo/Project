;
; Project.asm
;
; Created: 5/11/2024 11:29:53 AM
; Author : nsoo1
;
.equ PORTCDIR		= 0b11110000
.equ PORTBDIR		= 0b00001111
.equ INITCOLMASK	= 0b01111111
.equ INITROWMASK	= 0b00001000
.equ CHECKROWMASK	= 0b00001111

.equ KEYPADSIZE		= 4
.equ NSTATES		= 3
.equ LARGESTDIGIT	= 9

.def w				= r16				; free to change
.def cur_col		= r2				; free to change outside of keypad loop
.def cur_row		= r3				; free to change outside of keypad loop
.def keypad_size	= r4				; please don't change
.def row_val		= r5				; free to change outside of keypad loop
.def working_out	= r6				; free to change
.def arg0			= r10				; please don't change
.def arg1			= r11				; please don't change
.def arg2			= r12				; please don't change
.def row_mask		= r17				; please don't change
.def col_mask		= r18				; free to change outside of keypad loop
.def state			= r19				; please don't change
.def base			= r21				; please don't change

.cseg

.macro debounce
	ldi zl, 0b11111111
	ldi zh, 0b11111111

debounce_loop:
	nop
	subi zl, 1
	sbci zh, 0
	brne debounce_loop
.endmacro

	ldi		w, KEYPADSIZE				; initialise keypad size
	mov		keypad_size, w

	ldi		w, PORTCDIR					; initialise pin directions
	out		DDRC, w
	ldi		w, PORTBDIR
	out		DDRB, w

	clr		state						; initialise input state
	clr		arg0
	clr		arg1
	clr		arg2

scan_keypad:
	clr		cur_col						; clear column
	ldi		col_mask, INITCOLMASK		; initialise column mask

loop:
	cp		cur_col, keypad_size		; if maximum columns reached
	brlt	push_mask
	ldi		col_mask, INITCOLMASK		; reset the column mask
	clr		cur_col						; reset column count
push_mask:
	out		PORTC, col_mask
	ldi		w, 0b11111111				; delay to allow pin to update
update_delay:
	dec		w
	brne	update_delay

	in		w, PINC						; read portc
	andi	w, CHECKROWMASK				; read only the inputs
	cpi		w, CHECKROWMASK
	breq	nextcol						; check if any of the buttons are pressed

	mov		row_val, w					; save copy of inputs
	clr		cur_row						; initialise row scan
	ldi		row_mask, INITROWMASK
row_loop:
	mov		w, row_val					; get input value
	and		w, row_mask					; mask out single bit
	brne	inc_row						; if button pressed
	debounce							; debounce button
	in		w, PINC						; read port again
	and		w, row_mask					; check value of pin again
	breq	resolve						; if button still pressed resolve value
inc_row:
	lsr		row_mask					; shift row mask
	inc		cur_row						; increment row count
	cp		cur_row, keypad_size
	brne	row_loop
nextcol:
	lsr		col_mask					; shift column mask right
	sbr		col_mask, 0b10000000		; re-set left most bit
	inc		cur_col						; increment column count
	rjmp	loop

resolve:
	lsl		cur_row						; get key index by adding row * 4 + column
	lsl		cur_row
	mov		w, cur_row
	add		w, cur_col

	ldi		zh, high(value_lookup<<1)	; get numeric value from lookup table (or ascii if non-numeric)
	ldi		zl, low(value_lookup<<1)

	clr		r15							
	add		zl, w
	adc		zh, r15

	lpm		w, z

	; do stuff with keypad press

hold_loop:								; wait until button is unpressed before continuing to read keypad
	in		w, PINC						; read port again
	and		w, row_mask					; check value of pin again
	breq	hold_loop					; if button still pressed don't allow key to be continually read

	rjmp	scan_keypad

value_lookup:
	.db		1, 2, 3, 0x41, 4, 5, 6, 0x42, 7, 8, 9, 0x43, 0x2A, 0, 0x23, 0x44
ascii_lookup:
	.db		0x31, 0x32, 0x33, 0x41, 0x34, 0x35, 0x36, 0x42, 0x37, 0x38, 0x39, 0x43, 0x2A, 0x30, 0x23, 0x44