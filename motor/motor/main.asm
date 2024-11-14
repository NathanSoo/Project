;
; motor.asm
;
; Created: 11/11/2024 2:16:34 AM
; Author : ffordian
;

.include "m2560def.inc"

.def w            = r16	; Working register
.def motor_state  = r18 ; Stores state of motor (on/off)

; The macro clears a word (2 bytes) in data memory for the counter
; The parameter @0 is the memory address for that word
.macro clear_word
	ldi YL, low(@0)  ; load the memory address to Y
	ldi YH, high(@0)
	clr w
	st Y+, w		; clear the two bytes at @0 in SRAM
	st Y, w
.endmacro

.dseg 
SecondCounter:    .byte 2 ; Two byte counter for counting num_seconds
TempCounter:	  .byte 2 ; Temporary counter used to determine if 1 second has passed

.cseg
.org 0x0000
	rjmp reset
	nop

.org OVF0addr
	jmp timer0ovf

; Interrupt Service Routine for Timer 0 overflow
timer0ovf:
	push w
	in w, SREG
	push w
	push YH
	push YL
	push r25
	push r24						; Prologue ends
	ldi  YL, low(TempCounter) 
	ldi  YH, high(TempCounter)		; Y pointer points to temp counter
	ld   r24, Y+					; Load the value of the temporary counter.
	ld   r25, Y
	adiw r25:r24, 1					; Increase the temporary counter by one.
	cpi  r24, low(1000)				; Check if (r25:r24)=1000
	brne not_second
	cpi  r25, high(1000)
	brne not_second
	ldi  w, (1 << CS50)				; Toggle Timer 5 (no pre-scaling)
	eor  motor_state, w
	sts  TCCR5B, motor_state
	clear_word TempCounter
	ldi  YL, low(SecondCounter)		; Load the address of the second counter.
	ldi  YH, high(SecondCounter)		
	ld   r24, Y+					; Load the value of the second counter.
	ld   r25, Y
	adiw r25:r24, 1					; Increase the second counter by one.
	st   Y, r25						; Store the value of the second counter.
	st   -Y, r24
	rjmp endif
not_second:
	st   Y, r25
	st  -Y, r24
endif:
	pop  r24
	pop  r25
	pop  YL
	pop  YH
	pop  w
	out  SREG, w
	pop  w
	reti

reset:
	; Set Timer 0 to count seconds
	clr w
	out TCCR0A, w				    ; Set Timer 0 to Normal Mode
	ldi w, 0b00000011
	out TCCR0B, w					; Prescaler value = 64
	ldi temp, 1<<TOIE0				; Enable timer overflow interrupt
	sts TIMSK0, temp	
	sei								; Enable global interrupt
	; Set Timer 5 for waveform generation
	ldi temp, 0b00001000
	sts DDRL, temp					; Bit 3 will function as OC5A.
	clr temp						; the value controls the PWM duty cycle
	sts OCR5AH, temp				; Set higher byte of OCR5A to 0
	ldi temp, 0x4A        
	sts OCR5AL, temp				; Set lower byte to 0x4A
	; Set Timer5 to Phase Correct PWM mode.
	ldi temp, 0b00001001			; Set Timer clock frequency (no prescaling)
	sts TCCR5B, temp      
	ldi temp, (1<< WGM50)|(1<<COM5A1) ; Phase correct PWM mode
	sts TCCR5A, temp
	ldi motor_state, 0b00000001		; Init motor on
loop:
	rjmp loop
