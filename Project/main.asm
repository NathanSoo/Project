;
; Project.asm
;
; Created: 5/11/2024 11:29:53 AM
; Author : nsoo1
;

     ; Patient struct ;
;..........................;
;   Patient ID - 2 bytes   ;
;..........................;
;  Patient Name - 8 bytes  ;
;..........................;

.include "m2560def.inc"

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ STROBE = 1

.equ PORTCDIR				= 0b11110000
.equ PORTBDIR				= 0b00001111
.equ INITCOLMASK			= 0b01111111
.equ INITROWMASK			= 0b00001000
.equ CHECKROWMASK			= 0b00001111

.equ KEYPADSIZE				= 4
.equ NSTATES				= 3
.equ LARGESTDIGIT			= 9

.equ A						= 3
.equ B						= 7
.equ C						= 11
.equ D						= 15
.equ zero_key				= 13
.equ nine					= 10
.equ null					= 0xFF

.equ P_SIZE					= 10    ; Size of patient struct: 10 bytes
.equ INIT_ID				= 100   ; Patient IDs will start from 100

.equ max_name				= 8

.def w						= r16	; working register
.def entry_mode				= r2
.def new_name_cur			= r3
.def key_pressed			= r4
.def zero					= r5	; register always set to 0

.def last_key_pressed		= r6
.def times_key_pressed		= r7
.def is_timer_active		= r8

.def arg0					= r10
.def arg1					= r11
.def arg2					= r12

.dseg								; Start the data segment
.org 0x0200							; from address 0x0200

new_name:					.byte max_name
temp_name:					.byte 8            ; Queue functions assume 8 bytes for name
num_patients:				.byte 2
init_pid:					.byte 2                                  
first_patient:				.byte 2                 
last_patient:				.byte 2  ; Reserve space for Queue variables in data memory
is_keypad_timeout:			.byte 1


.cseg
.org 0x0000
	rjmp	reset
	nop

.org OVF3addr
	jmp		keypad_timeout

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

; Reads name from temp_name variable in data memory,
; stores it in newest patient struct on queue.
; Assumes address of patient_name stored in Y.
.macro read_name
	ldi ZH, high(temp_name)
	ldi ZL, low(temp_name)
	ldi r26, 8
read_loop:
	ld  r27, Z+
	st  Y+, r27
	dec r26
	cpi r26, 0
	brne read_loop                     ; Continue reading up to 8 chars
.endmacro 

; init patient queue
; call first before any registers are used
.macro init_queue
	ldi  YH, high(num_patients)                      
	ldi  YL, low(num_patients)
	clr  r24
	clr  r25
	st   Y+, r24                       ; Init num_patients = 0
	st   Y+, r25
	ldi  r24, INIT_ID-1				   ; Init patient ID
	st   Y, r24
	adiw Y, 2
    mov  r24, YL
	mov  r25, YH
	adiw r24, 8                        ; Adjust num bytes here if needed
	st   Y+, r24
	st   Y+, r25                       ; init pointer to first patient
    ldi  r25, high(init_pid)
	ldi  r24, low(init_pid)
	st   Y+, r24
	st   Y, r25                        ; init pointer to last patient
.endmacro

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
; decimal_conversion	: shouldn't need to use this since i integrated the decimal conversion into all the LCD places,
;						: ENTRY_MESSAGE and DISPLAY_NUMBER_RIGHT should cover all use cases for displaying numbers on LCD
; Replace with your application code
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
	ldi r16, 20
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

;______________________________________________________________________
; INITIALISE
;______________________________________________________________________
reset:
	init_queue
	rcall	INITIALISE_LCD
	rcall	LCD_DISPLAY_MODE

	clr		zero

	ldi		w, PORTCDIR					; initialise pin directions
	out		DDRC, w
	ldi		w, PORTBDIR
	out		DDRB, w

	ldi		w, null						; initialise last key pressed
	mov		last_key_pressed, w

	clr		times_key_pressed			; initialise times last key pressed
	clr		entry_mode

	ldi		w, high(new_name)
	mov		arg0, w

	ldi		w, low(new_name)
	mov		arg1, w

	ldi		w, max_name
	mov		arg2, w

	rcall	initialise_array

	clr		new_name_cur
	clr		is_timer_active

	ldi		w, 0b00000000
	sts		TCCR3A, w
	ldi		w, 0b00000000
	sts		TCCR3B, w

	sts		TIMSK3, zero

	ldi		zh, high(is_keypad_timeout)
	ldi		zl, low(is_keypad_timeout)
	st		z, zero
	;ldi		w, (1 << TOIE1)
	;sts		TIMSK1, w

	sei

;______________________________________________________________________
; MAIN LOOP
;______________________________________________________________________
loop:
	rcall	scan_keypad					; poll until key is pressed then return key index in r24
	mov		key_pressed, r24

	sts		TCCR3B, zero
	sts		TIMSK3, zero					; disable keypad timeout temporarily

	ldi		zh, high(is_keypad_timeout)
	ldi		zl, low(is_keypad_timeout)
	ld		w, z
	cp		w, zero
	breq	no_timeout
	st		z, zero
	inc		new_name_cur					; new_name_cur++
	ldi		w, null
	mov		last_key_pressed, w				; last_key_pressed = null
	clr		times_key_pressed				; times_key_pressed = 0
	clr		is_timer_active


no_timeout:
	;	if entry_mode is false and key_pressed == 'A'
	;		enter entry mode						
	cp		entry_mode, zero
	brne	not_display_mode
	ldi		w, A
	cp		key_pressed, w
	brne	not_display_mode
	inc		entry_mode
	rcall	LCD_ENTRY_MODE
	rcall	DISPLAY_BOTTOM_LEFT
	rjmp	end_process_key

not_display_mode:
	;	else if entry_mode is true
	;		handle keys which are not 'A'
	cp		entry_mode, zero
	breq	to_end_process_key
	;		if new_name_cur != 0 and key_pressed == 'B'
	;			backspace button is pressed
	cp		new_name_cur, zero
	breq	not_backspace
	ldi		w, B
	cp		key_pressed, w
	brne	not_backspace
	clr		is_timer_active					; keypad_timer_stop()
	dec		new_name_cur					; new_name_cur --
	
	ldi		zh, high(new_name)				; new_name[new_name_cur] = 0
	ldi		zl, low(new_name)

	add		zl, new_name_cur
	adc		zh, zero

	st		z, zero


	rcall	BACKSPACE

	ldi		w, null							; last_key_pressed = null
	mov		last_key_pressed, w
	clr		times_key_pressed				; times_key_pressed = 0
	rjmp	end_process_key

not_backspace:
	;		else if key_pressed == 'C'
	;			clear button is pressed
	ldi		w, C
	cp		key_pressed, w
	brne	not_clear
	clr		is_timer_active					; keypad_timer_stop()
	clr		new_name_cur					; new_name_cur = 0
	
	rcall	initialise_array
	rcall	LCD_ENTRY_MODE
	rcall	DISPLAY_BOTTOM_LEFT

	ldi		w, null							; last_key_pressed = null
	mov		last_key_pressed, w
	clr		times_key_pressed				; times_key_pressed = 0
to_end_process_key:
	rjmp	end_process_key

not_clear:
	;		else if key_pressed == 'D'
	;			enter button is pressed
	ldi		w, D
	cp		key_pressed, w
	brne	not_enter

	rcall	enqueue
	rcall	LCD_DISPLAY_MODE

	clr		is_timer_active					; keypad_timer_stop()
	ldi		w, high(new_name)				; initialise_array(new_name, 8)
	mov		arg0, w

	ldi		w, low(new_name)
	mov		arg1, w

	ldi		w, max_name
	mov		arg2, w

	rcall	initialise_array

	clr		new_name_cur					; new_name_cur = 0
	ldi		w, null							; last_key_pressed = null
	mov		last_key_pressed, w
	clr		times_key_pressed				; times_key_pressed = 0
	clr		entry_mode						; entry_mode = false
	rjmp	end_process_key

not_enter:
	;		else if key_pressed == 'A'
	;			do nothing
	ldi		w, A
	cp		key_pressed, w
	brne	not_nothing1
	rjmp	end_process_key

not_nothing1:
	;		else if key_pressed is on the bottom row
	;			do nothing
	ldi		w, C
	inc		w
	cp		key_pressed, w
	brlo	not_nothing2
	rjmp	end_process_key

not_nothing2:
	;		else if key_pressed == '1'
	;			do nothing
	cp		key_pressed, zero
	brne	not_nothing3
	rjmp	end_process_key

not_nothing3:
	;		else if new_name_cur < 8
	ldi		w, max_name
	cp		new_name_cur, w
	brsh	to_end_process_key
	clr		is_timer_active					; keypad_timer_restart()
	inc		is_timer_active
	sts		TCNT3h, zero
	sts		TCNT3l, zero
	;			if times_key_pressed == 0
	;				first input key pressed
	cp		times_key_pressed, zero
	brne	not_first_key
	mov		last_key_pressed, key_pressed	; last_key_presed = key_pressed
	inc		times_key_pressed				; times_key_pressed = 1
	rjmp	end_process_key

not_first_key:
	;			else if last_key_pressed == key_pressed
	;				cycle between letters on the same key
	cp		last_key_pressed, key_pressed
	brne	not_same_key
	inc		times_key_pressed
	;				if key_pressed == '9' and times_key_pressed == 5
	;					wrap the number of times key is pressed
	ldi		w, nine
	cp		key_pressed, w
	brne	not_nine_wrap
	ldi		w, 5
	cp		times_key_pressed, w
	brne	not_nine_wrap
	clr		times_key_pressed				; times_key_pressed = 1
	inc		times_key_pressed
	rjmp	end_key_wrap
	;				else if key_pressed != '9' and times_key_pressed == 4 
	;					wrap the number of times key is pressed
not_nine_wrap:
	ldi		w, nine
	cp		key_pressed, w
	breq	end_key_wrap
	ldi		w, 4
	cp		times_key_pressed, w
	brne	end_key_wrap
	clr		times_key_pressed				; times_key_pressed = 1
	inc		times_key_pressed
end_key_wrap:
	rjmp	end_process_key

not_same_key:
	;			else
	;				enter letter
	ldi		w, max_name
	cp		new_name_cur, w
	brsh	end_process_key

	ldi		zh, high(new_name)				; new_name[new_name_cur] = keypad_to_ascii(last_key_pressed, times_key_pressed)
	ldi		zl, low(new_name)

	add		zl, new_name_cur
	adc		zh, zero

	mov		arg0, last_key_pressed
	mov		arg1, times_key_pressed
	rcall	keypad_to_ascii

	st		z, r24

	mov		r19, r24
	rcall	WRITE

	inc		new_name_cur					; new_name_cur ++
	mov		last_key_pressed, key_pressed	; last_key_pressed = key_pressed
	clr		times_key_pressed				; times_key_pressed = 1
	inc		times_key_pressed
end_process_key:

	cp		is_timer_active, zero
	breq	timer_not_active
	ldi		w, 0b00000100
	sts		TCCR3B, w
	ldi		w, (1 << TOIE3)
	sts		TIMSK3, w
timer_not_active:

	out		PORTB, is_timer_active

	rjmp	loop

;______________________________________________________________________
; HELPER FUNCTIONS
;______________________________________________________________________

;______________________________________________________________________
; initialise_array
; arguments
; array - arg0:arg1 (r10:r11)
; length - arg2 (r12)
;______________________________________________________________________
initialise_array:
	push	zl
	push	zh
	push	r17

	mov		zh, arg0
	mov		zl, arg1

	mov		r17, arg2

	;rcall	INITIALISE_LCD

initialise_array_loop:
	cp		r17, zero
	breq	end_initialise_array_loop
	st		z+, zero

	dec		r17
	rjmp	initialise_array_loop
end_initialise_array_loop:

	pop		r17
	pop		zh
	pop		zl
	ret

;______________________________________________________________________
; keypad_to_ascii
; arguments
; last_key_pressed - arg0 (r10)
; times_key_pressed - arg1 (r11)
;______________________________________________________________________
keypad_to_ascii:
	push	zl
	push	zh
	push	w
	push	r17

	; get keypad number value from lookup table
	ldi		zh, high(value_lookup << 1)
	ldi		zl, low(value_lookup << 1)

	add		zl, arg0
	adc		zh, zero

	; do maths
	lpm		w, z
	dec		w
	dec		w
	dec		arg1

	mov		r17, w
	lsl		r17
	add		w, r17
	add		w, arg1

	; get ascii value of input character from lookup table
	ldi		zh, high(ascii_lookup << 1)
	ldi		zl, low(ascii_lookup << 1)

	add		zl, w
	adc		zh, zero

	lpm		w, z
	mov		r24, w

	pop		r17
	pop		w
	pop		zh
	pop		zl
	ret

;______________________________________________________________________
; keypad_to_ascii
; arguments
; last_key_pressed - arg0 (r10)
; times_key_pressed - arg1 (r11)
;______________________________________________________________________
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

; Clears temp_name variable in memory
clear_name:
	push YL
	push YH
	push r16
	push r17
	ldi  YH, high(temp_name)
	ldi  YL, low(temp_name)
	ldi  r16, 8
	ldi  r17, 0
delete:
	st   Y+, r17
	dec  r16
	cpi  r16, 0
	brne delete
	pop  r17
	pop  r16
	pop  ZH
	pop  ZL
	ret

; Enqueues new patient
; Assigns pid and returns pointer to location of
; patient struct in data memory.
; Should ideally be done with interrupts disabled
; to avoid race conditions.
enqueue:
	push YL
	push YH
	push ZL
	push ZH
	push r26
	push r27
	ldi  YH, high(last_patient)
	ldi  YL, low(last_patient)
	ld   r26, Y+
	ld   r27, Y
	movw Z, r26                    ; Store pointer to previous end of Q
	adiw r26, P_SIZE
	st   Y, r27
	st   -Y, r26                   ; Update pointer to last patient in queue
	movw Y, r26                    ; Move Y pointer to new_patient
	movw r24, r26				   ; Move address of new_patient to return registers
	ld   r26, Z+
	ld   r27, Z                    ; Load most recent pid
	adiw r26, 1					   ; Increment
	st   Y+, r26
	st   Y+, r27			       ; Assign new pid to next patient
	read_name					   ; Reads in patient name from data memory
	rcall clear_name
	ldi  YH, high(num_patients)
	ldi  YL, low(num_patients)  
	ld   r26, Y+
	ld   r27, Y
	adiw r26, 1                 
	st   Y, r27
	st   -Y, r26				   ; Update num_patients last                     
	pop  r27
	pop  r26
	pop  ZH
	pop  ZL
	pop  YH
	pop  YL
	ret

; Dequeues first patient from queue and discards
; Does not return any values.
; Should ideally be done with interrupts disabled
; to avoid race conditions.	
dequeue:
	push YH
	push YL
	push r25
	push r24
	push r27
	push r26
	ldi  YH, high(num_patients)
	ldi  YL, low(num_patients)      
	ld   r26, Y+
	ld   r27, Y
	sbiw r26, 1
	st   Y, r27
	st  -Y, r26                     ; Update num_patients first
	ldi  YH, high(first_patient)    
	ldi  YL, low(first_patient)
	ld   r24, Y+					
	ld   r25, Y						; Store address of dequeued patient to be cleared
	cpi  r26, 0
	brne not_empty
	cpi  r27, 0
	brne not_empty                  ; If queue not empty, proceed
	ldi  YH, high(first_patient)    
	ldi  YL, low(first_patient)
	movw r26, Y                     ; Else reset queue variables
	adiw r26, 8
	st   Y+, r26
	st   Y+, r27
	ldi  r27, high(init_pid)
	ldi  r26, low(init_pid)
	st   Y+, r26
	st   Y, r27
	rjmp clear_patient
not_empty:
	ldi  YH, high(first_patient)    ; Increment first_patient pointer
	ldi  YL, low(first_patient)
	ld   r26, Y+
	ld   r27, Y
	adiw r26, P_SIZE
	st   Y, r27
	st   -Y, r26
clear_patient:
	movw Y, r24                     
	ldi r26, P_SIZE                 ; Then clear patient struct
	ldi r27, 0
clear_pt_loop:
	st  Y+, r27
	dec r26
	cpi r26, 0
	brne clear_pt_loop
	pop r26
	pop r27
	pop r24
	pop r25
	pop YL
	pop YH
	ret



keypad_timeout:
	push	zl
	push	zh
	push	arg0
	push	arg1
	push	r24
	push	r19
	push	w
	push	zero

	in		w, sreg

	push	w

	clr		zero
	sts		TCCR3B, zero
	sts		TIMSK3, zero					; disable_keypad_timer()

	ldi		zh, high(new_name)				; new_name[new_name_cur] = keypad_to_ascii(last_key_pressed, times_key_pressed)
	ldi		zl, low(new_name)

	add		zl, new_name_cur
	adc		zh, zero

	mov		arg0, last_key_pressed
	mov		arg1, times_key_pressed
	rcall	keypad_to_ascii

	st		z, r24

	mov		r19, r24
	rcall	WRITE

	ldi		zh, high(is_keypad_timeout)
	ldi		zl, low(is_keypad_timeout)
	ldi		w, 1
	st		z, w

	pop		w

	out		sreg, w

	pop		zero
	pop		w
	pop		r19
	pop		r24
	pop		arg1
	pop		arg0
	pop		zh
	pop		zl
	reti
	
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

	ldi r16, 0b00001110	; LCD display on, no cursor, blink
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

value_lookup:
	.db		1, 2, 3, 0x41, 4, 5, 6, 0x42, 7, 8, 9, 0x43, 0x2A, 0, 0x23, 0x44
ascii_lookup:
	.db		'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', 0