;
; lab5_output.asm
;
; Created: 22/10/2024 9:06:18 PM
; Author : Owen
;

; all subroutines expects parameters in r19
; Sub routines:
; INTIALISE_LCD			: initialises LCD display
; LCD_DISPLAY_MODE		: clears the LCD display and sets it to display mode
; DISPLAY_BOTTOM_LEFT	: moves the cursor to bottom left
; DISPLAY_NUMBER_RIGHT	: takes unsigned int as parameter in r19 and displays bottom right aligned number
; LCD_ENTRY_MODE		: clears the LCD display and sets it to entry mode
; ENTRY_MESSAGE			: takes unsigned int as parameter in r19 and displays descriptive message showing patient number
; BACKSPACE				: delete 1 char from patient input
; WRITE					: write 1 char to LCD at current cursor position
; display_front_patient	: peeks at the front of the queue and writes the patient name and pID to LCD display mode
; decimal_conversion	: shouldn't need to use this since i integrated the decimal conversion into all the LCD places,
;						: ENTRY_MESSAGE and DISPLAY_NUMBER_RIGHT should cover all use cases for displaying numbers on LCD
; Replace with your application code
.include "m2560def.inc"
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ STROBE = 1

.macro set_LCD_RS
	sbi PORTA, LCD_RS
.endmacro
.macro clr_LCD_RS
	cbi PORTA, LCD_RS
.endmacro

.macro set_LCD_E
	sbi PORTA, LCD_E
.endmacro
.macro clr_LCD_E
	cbi PORTA, LCD_E
.endmacro

.macro set_LCD_RW
	sbi PORTA, LCD_RW
.endmacro
.macro clr_LCD_RW
	cbi PORTA, LCD_RW
.endmacro

.macro set_strobe
	sbi PORTA, STROBE
.endmacro
.macro clr_strobe
	cbi PORTA, STROBE
.endmacro

.macro lcd_write_cmd		; set LCD instructions, does not wait for BF
	out PORTF, r16			; set r16 port
	clr_LCD_RS
	clr_LCD_RW				; RS = 0, RW = 0 for a command write
	nop
	set_LCD_E		
	nop
	nop
	nop
	clr_LCD_E
	nop
	nop
	nop
.endmacro

.macro lcd_write_data		; write r16 to LCD, waits for BF
	out PORTF, r19			; set r16 port
	set_LCD_RS
	clr_LCD_RW				; RS = 1, RW = 0 for r16 write
	nop
	set_LCD_E		;
	nop
	nop
	nop
	clr_LCD_E
	nop
	nop
	nop
.endmacro

.macro lcd_wait_busy		; read from LCD until BF is clear
	clr r17
	out DDRF, r17			; LCD port input
	clr_LCD_RS
	set_LCD_RW				; RS = 0, RW = 1, cmd port read
busy:
	nop						
	set_LCD_E				; turn on enable pin
	nop						; r16 delay
	nop
	nop
	in r17, PINF			; read value from LCD
	clr_LCD_E				; clear enable pin
	sbrc r17, 7			; skip next if busy flag not set
	rjmp busy				; else loop

	nop
	nop
	nop
	clr_LCD_RS
	clr_LCD_RW				; RS, RW = 0, IR write
	ser r17
	out DDRF, r17			; LCD port output
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

; CAN DELETE
.macro long_delay
	ldi r16, 100
l1:
	ldi r18, low(65000)		; delay 65ms
	ldi r19, high(65000)
	delay
	dec r16
	brne loop_jmp
	rjmp no_jmp
loop_jmp:
	rjmp l1
no_jmp:
.endmacro

; example use CAN DELETE
main:
	; display mode
	rcall INITIALISE_LCD
	rcall LCD_DISPLAY_MODE
	rcall DISPLAY_BOTTOM_LEFT
	ldi r19, 'O'
	rcall WRITE
	ldi r19, 'W'
	rcall WRITE
	ldi r19, 'E'
	rcall WRITE
	ldi r19, 'N'
	rcall WRITE

	ldi r19, 99
	rcall DISPLAY_NUMBER_RIGHT

	long_delay
	; entry mode
	rcall LCD_ENTRY_MODE
	rcall DISPLAY_BOTTOM_LEFT
	ldi r19, 'O'
	rcall WRITE
	ldi r19, 'W'
	rcall WRITE
	ldi r19, 'E'
	rcall WRITE
	ldi r19, 'N'
	rcall WRITE

	long_delay
	; descriptive message
	ldi r19, 99
	rcall ENTRY_MESSAGE
stop:
	rjmp stop

; initialise the LCD
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
	out DDRF, r17			; LCD r16
	out DDRA, r17			; LCD Ctrl
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

	ldi r16, 0b00001110	; LCD display on, cursor on, blink
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

; clears the LCD display and sets it to display mode
LCD_DISPLAY_MODE:
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
	set_strobe				; STROBE on
	
	ldi r16, 0b00000001	; clear display
	lcd_write_cmd
	lcd_wait_busy

	ldi r19, 'N'			; display "Next Patient:"
	rcall WRITE
	ldi r19, 'e'
	rcall WRITE
	ldi r19, 'x'
	rcall WRITE
	ldi r19, 't'
	rcall WRITE
	ldi r19, ' '
	rcall WRITE
	ldi r19, 'P'
	rcall WRITE
	ldi r19, 'a'
	rcall WRITE
	ldi r19, 't'
	rcall WRITE
	ldi r19, 'i'
	rcall WRITE
	ldi r19, 'e'
	rcall WRITE
	ldi r19, 'n'
	rcall WRITE
	ldi r19, 't'
	rcall WRITE
	ldi r19, ':'
	rcall WRITE
	
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

; shifts cursor to bottom left of screen
DISPLAY_BOTTOM_LEFT:
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

	ldi r16, 0b11000000	; Set DDRAM address to bottom left
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

;expects unsigned int as parameter in r19 and displays bottom right aligned number
DISPLAY_NUMBER_RIGHT:
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

	ldi r16, 0b11001111	; Set DDRAM address to bottom right
	lcd_write_cmd
	lcd_wait_busy
	ldi r16, 0b00000110	; INCREMENT, no shift
	lcd_write_cmd
	lcd_wait_busy
	
	cpi r19, 10
	brsh tens
	rjmp display
tens:
	ldi r16, 0b00010000	; shift cursor left 1
	lcd_write_cmd
	lcd_wait_busy
	cpi r19, 100
	brsh hundreds
	rjmp display
hundreds:
	ldi r16, 0b00010000	; shift cursor left 1
	lcd_write_cmd
	lcd_wait_busy
display:
	mov r24, r19
	rcall decimal_conversion
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

; clears LCD display and sets it to entry mode
LCD_ENTRY_MODE:
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
	clr_strobe				; STROBE on
	
	ldi r16, 0b00000001	; clear display
	lcd_write_cmd
	lcd_wait_busy

	ldi r19, 'E'			; display "Next Patient:"
	rcall WRITE
	ldi r19, 'n'
	rcall WRITE
	ldi r19, 't'
	rcall WRITE
	ldi r19, 'e'
	rcall WRITE
	ldi r19, 'r'
	rcall WRITE
	ldi r19, ' '
	rcall WRITE
	ldi r19, 'N'
	rcall WRITE
	ldi r19, 'a'
	rcall WRITE
	ldi r19, 'm'
	rcall WRITE
	ldi r19, 'e'
	rcall WRITE
	ldi r19, ':'
	rcall WRITE
	
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

; expects unsigned int as parameter in r19 and displays descriptive message to patient
ENTRY_MESSAGE:
	; prologue
	push r16
	push r17
	push r18
	push r19
	push r24
	push YL
	push YH
	in YL, SPL
	in YH, SPH
	sbiw Y, 1

	ldi r16, 0b00000001	; clear display
	lcd_write_cmd
	lcd_wait_busy

	mov r24, r19

	ldi r19, 'Y'
	rcall WRITE
	ldi r19, 'o'
	rcall WRITE
	ldi r19, 'u'
	rcall WRITE
	ldi r19, ' '
	rcall WRITE
	ldi r19, 'a'
	rcall WRITE
	ldi r19, 'r'
	rcall WRITE
	ldi r19, 'e'
	rcall WRITE
	ldi r19, ' '
	rcall WRITE
	ldi r19, 'p'
	rcall WRITE
	ldi r19, 'a'
	rcall WRITE
	ldi r19, 't'
	rcall WRITE
	ldi r19, 'i'
	rcall WRITE
	ldi r19, 'e'
	rcall WRITE
	ldi r19, 'n'
	rcall WRITE
	ldi r19, 't'
	rcall WRITE

	rcall DISPLAY_BOTTOM_LEFT

	ldi r19, 'n'
	rcall WRITE
	ldi r19, 'u'
	rcall WRITE
	ldi r19, 'm'
	rcall WRITE
	ldi r19, 'b'
	rcall WRITE
	ldi r19, 'e'
	rcall WRITE
	ldi r19, 'r'
	rcall WRITE
	ldi r19, ':'
	rcall WRITE
	ldi r19, ' '
	rcall WRITE
	rcall decimal_conversion
	;epilogue
	adiw Y, 1
	out SPH, YH
	out SPL, YL
	pop YH
	pop YL
	pop r24
	pop r19
	pop r18
	pop r17
	pop r16
	ret

; delete 1 character from patient input in LCD entry mode
BACKSPACE:
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

	ldi r16, 0b00010000	; shift cursor left 1
	lcd_write_cmd
	lcd_wait_busy

	ldi r19, ' '			; delete letter
	rcall WRITE

	ldi r16, 0b00010000	; shift cursor left 1
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

; expects an ASCII character in r19 and displays to current cursor position
WRITE:
	; write to r16

	push r16
	push r17
	push YL
	push YH
	in YL, SPL
	in YH, SPH
	sbiw Y, 1

	lcd_write_data
	lcd_wait_busy

	;epilogue
	adiw Y, 1
	out SPH, YH
	out SPL, YL
	pop YH
	pop YL
	pop r17
	pop r16
	ret

; expects digit in r19, ASCII '0' in r23
.macro display_digit
	add r19, r23              ; Convert to ASCII ('0' = 0x30)
	rcall WRITE
.endmacro
;expects dividend in r26 and divisor in r25
.macro divide
	clr r27                   ; Clear quotient register
divide_loop:
    cp r26, r25               ; Compare r26 (dividend) with r25 (divisor)
    brlo divide_done           ; If r26 < r25, exit loop

    sub r26, r25              ; Subtract divisor from dividend
    inc r27                   ; Increment quotient
    rjmp divide_loop           ; Repeat until r26 < r28
divide_done:
	nop
.endmacro
; parameter passed into r24
; conflict registers: 23, 25, 26, 27, 19
decimal_conversion:
;prologue
	push r19
	push r23
	push r24
	push r25
	push r26
	push r27
	push YL
	push YH
	in YL, SPL
	in YH, SPH
	sbiw Y, 1

	ldi r23, '0'				; load ASCII '0'
	mov r26, r24				; Load result (y) r24 into register r26.

	ldi r25, 100				; Load 100 for division
    divide				; Call divide subroutine
	mov r19, r27				; Get the quotient (hundreds digit) in r19
    cpi r19, 0					; Check if it's zero
	brne from_hundreds
	rjmp no_hundreds

from_hundreds:
	display_digit
	ldi r25, 10					; Load 10 for division
    divide				; Call divide subroutine
	mov r19, r27				; move quotient to r19
	display_digit				; always display tens if from hundreds
	rjmp display_ones_digit

no_hundreds:
	ldi r25, 10					; Load 10 for division
    divide				; Call divide subroutine
	cpi r27, 0                ; Check if it's zero
	breq display_ones_digit
	mov r19, r27
	display_digit
	rjmp display_ones_digit

display_ones_digit:
	mov r19, r26
	display_digit

	;epilogue
	adiw Y, 1
    out SPH, YH
    out SPL, YL
    pop YH
    pop YL
    pop r27
    pop r26
    pop r25
	pop r24
    pop r23
    pop r19
    ret

; display the front patient in data memory to LCD display mode
display_front_patient:
	push YL
	push YH
	push ZL
	push ZH
	push r16
	push r19
	in YL, SPL
	in YH, SPH
	sbiw Y, 1

	ldi ZH, high(first_patient)
	ldi ZL, low(first_patient)
	ld r19, Z+						; load lower byte of pID into r19
	rcall DISPLAY_NUMBER_RIGHT		; display bottom righ aligned pID
	adiw Z, 1						; skip over higher byte of pID
	; loop over reamining 8 bytes of chars
	ldi r16, 8
display_loop:
	ld r19, Z+
	rcall write
	dec r16
	cpi r16, 0
	brne display_loop
	;epilogue
	adiw Y, 1
	out SPH, YH
	out SPL, YL
	pop r19
	pop r16
	pop ZH
	pop ZL
	pop YH
	pop YL
	ret
