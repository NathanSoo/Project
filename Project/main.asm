;
; Project.asm
;
; Created: 5/11/2024 11:29:53 AM
; Author : nsoo1
;
.include "m2560def.inc"

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5

.equ PORTCDIR		= 0b11110000
.equ PORTBDIR		= 0b00001111
.equ INITCOLMASK	= 0b01111111
.equ INITROWMASK	= 0b00001000
.equ CHECKROWMASK	= 0b00001111

.equ KEYPADSIZE		= 4
.equ NSTATES		= 3
.equ LARGESTDIGIT	= 9

.def w				= r16

.cseg


.macro lcd_write_cmd		; set LCD instructions, does not wait for BF
	out PORTF, r16			; set data port
	clr r18
	out PORTA, r18			; RS = 0, RW = 0 for a command write
	nop
	sbi PORTA, LCD_E		
	nop
	nop
	nop
	cbi PORTA, LCD_E
	nop
	nop
	nop
.endmacro

.macro lcd_write_data		; write data to LCD, waits for BF
	out PORTF, r27			; set data port
	ldi r18, (1 << LCD_RS)|(0 << LCD_RW)
	out PORTA, r18			; RS = 1, RW = 0 for data write
	nop
	sbi PORTA, LCD_E		;
	nop
	nop
	nop
	cbi PORTA, LCD_E
	nop
	nop
	nop
.endmacro

.macro lcd_wait_busy		; read from LCD until BF is clear
	clr r18
	out DDRF, r17			; read from LCD
	ldi r18, (0 << LCD_RS)|(1 << LCD_RW)
	out PORTA, r18			; RS = =, RW = 1, cmd port read
busy:
	nop						
	sbi PORTA, LCD_E		; turn on enable pin
	nop						; data delay
	nop
	nop
	in r18, PINF			; read value from LCD
	cbi PORTA, LCD_E		; clear enable pin
	sbrc r18, 7			; skip next if busy flag not set
	rjmp busy				; else loop

	nop
	nop
	nop
	clr r18
	out PORTA, r18			; RS, RW = 0, IR write
	ser r18
	out DDRF, r18			; output to LCD
	nop
	nop
	nop
.endmacro

.macro delay				; delay for 1us
loop1:
	ldi r17, 3				; 1
loop2:
	dec r17				; 1
	nop						; 1
	brne loop2				; 2 taken, 1 not ----> inner loop total is 11 cycles
	subi r18, 1				; 1
	sbci r19, 0				; 1
	brne loop1				; 2 taken, each outer iteration is 11 + 1 + 1 + 1 + 2 = 16 clock cycles at 16Mhz = 1us
.endmacro

; changes registers:
; zl
; zh
.macro debounce
	ldi zl, 0b11111111
	ldi zh, 0b11111111

debounce_loop:
	nop
	subi zl, 1
	sbci zh, 0
	brne debounce_loop
.endmacro

	rcall	INITIALISE_LCD

	ldi		w, PORTCDIR					; initialise pin directions
	out		DDRC, w
	ldi		w, PORTBDIR
	out		DDRB, w

;______________________________________________________________________
; MAIN LOOP
;______________________________________________________________________
loop:
	rcall	scan_keypad
	mov		r27, r24
	rcall	INITIALISE_LCD
	lcd_write_data
	lcd_wait_busy
	rjmp	loop


INITIALISE_LCD:
	; prologue
	push r16
	push r17
	push r18
	push r19
	push YL
	push YH
	in YL, SPL
	in YH, SPH
	sbiw Y, 1

	ser r17
	out DDRF, r17
	out DDRA, r17
	clr r17
	out PORTF, r17
	out PORTA, r17
	;
	ldi r18, low(15000)		; delay 15ms
	ldi r19, high(15000)
	delay
	ldi r16, 0b00111000	; 2 x 5 x 7 r18 = 1, 8bits | N = 1, 2-line | F = 0, 5 x 7 dots
	lcd_write_cmd			; 1st function cmd set

	ldi r18, low(4100)		; delay 4.1ms
	ldi r19, high(4100)
	delay
	lcd_write_cmd			; 2nd function cmd set

	ldi r18, low(100)		; delay 4.1ms
	ldi r19, high(100)
	delay
	lcd_write_cmd			; 3rd function cmd set
	lcd_write_cmd			; final function cmd set
	;
	lcd_wait_busy			; wait until ready

	ldi r16, 0b00001000	; LCD display off
	lcd_write_cmd
	lcd_wait_busy

	ldi r16, 0b00000001	; LCD display clear
	lcd_write_cmd
	lcd_wait_busy

	ldi r16, 0b00000110	; increment, no shift
	lcd_write_cmd
	lcd_wait_busy

	ldi r16, 0b00001111	; LCD display on, cursor, blink
	lcd_write_cmd
	lcd_wait_busy

	;epilogue
	adiw Y, 1
	out SPH, YH
	out SPL, YL
	pop YH
	pop YL
	pop r19
	pop r18
	pop r17
	pop r16
	ret

scan_keypad:
	push	w
	push	r2
	push	r3
	push	r4
	push	r5
	push	r17
	push	r18

	push	zl
	push	zh

	ldi		w, KEYPADSIZE				; initialise keypad size
	mov		r4, w
	clr		r2							; clear column
	ldi		r18, INITCOLMASK		; initialise column mask

col_loop:
	cp		r2, r4		; if maximum columns reached
	brlt	push_mask
	ldi		r18, INITCOLMASK		; reset the column mask
	clr		r2						; reset column count
push_mask:
	out		PORTC, r18
	ldi		w, 0b11111111				; delay to allow pin to update
update_delay:
	dec		w
	brne	update_delay

	in		w, PINC						; read portc
	andi	w, CHECKROWMASK				; read only the inputs
	cpi		w, CHECKROWMASK
	breq	nextcol						; check if any of the buttons are pressed

	mov		r5, w					; save copy of inputs
	clr		r3						; initialise row scan
	ldi		r17, INITROWMASK
row_loop:
	mov		w, r5					; get input value
	and		w, r17					; mask out single bit
	brne	inc_row						; if button pressed
	debounce							; debounce button
	in		w, PINC						; read port again
	and		w, r17					; check value of pin again
	breq	resolve						; if button still pressed resolve value
inc_row:
	lsr		r17					; shift row mask
	inc		r3						; increment row count
	cp		r3, r4
	brne	row_loop
nextcol:
	lsr		r18					; shift column mask right
	sbr		r18, 0b10000000		; re-set left most bit
	inc		r2						; increment column count
	rjmp	col_loop

resolve:
	lsl		r3						; get key index by adding row * 4 + column
	lsl		r3
	mov		w, r3
	add		w, r2

	ldi		zh, high(ascii_lookup<<1)	; get numeric value from lookup table (or ascii if non-numeric)
	ldi		zl, low(ascii_lookup<<1)

	clr		r4							
	add		zl, w
	adc		zh, r4

	lpm		w, z

	mov		r24, w

hold_loop:								; wait until button is unpressed before continuing to read keypad
	in		w, PINC						; read port again
	and		w, r17					; check value of pin again
	breq	hold_loop					; if button still pressed don't allow key to be continually read

	pop		zh
	pop		zl

	pop		r18
	pop		r17
	pop		r5
	pop		r4
	pop		r3
	pop		r2
	pop		w
	ret

value_lookup:
	.db		1, 2, 3, 0x41, 4, 5, 6, 0x42, 7, 8, 9, 0x43, 0x2A, 0, 0x23, 0x44
ascii_lookup:
	.db		0x31, 0x32, 0x33, 0x41, 0x34, 0x35, 0x36, 0x42, 0x37, 0x38, 0x39, 0x43, 0x2A, 0x30, 0x23, 0x44