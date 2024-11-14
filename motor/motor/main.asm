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
	ldi  w, (1<<COM5A1)	            ; Toggle motor
	eor  motor_state, w
	sts  TCCR5A, motor_state
	cpi  motor_state, 0
	brne end_interrupt
	ldi  w, 0
	sts  TCNT5H, w
	sts  TCNT5L, w
end_interrupt:
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

; Motor on for 1 second, off 1 second
beep_beep:
	push w
	clear_word SecondCounter
	clear_word TempCounter
	ldi  motor_state, (1<<COM5A1)   ; Init motor on
	ldi  w, (1<< WGM50)|(1<<COM5A1)	; Set Timer 5 to Phase Correct PWM mode
	sts  TCCR5A, w
	pop  w
	ret

reset:
	; Set Timer 0 to count seconds
	clr w
	out TCCR0A, w				    ; Set Timer 0 to Normal Mode
	ldi w, 0b00000011
	out TCCR0B, w					; Prescaler value = 64
	ldi w, 1<<TOIE0					; Enable Timer 0 overflow interrupt
	sts TIMSK0, w
	sei                             ; Enable global interrupts
	; Set Timer 5 for waveform generation
	ldi w, 0b00001000
	sts DDRL, w						; Set Pin 3, Port L as output pin
	ldi w, (1 << CS50)				; Toggle Timer 5 (no pre-scaling)
	sts TCCR5B, w
	clr w
	sts OCR5AH, w					; Set timer compare value to 0x4A
	ldi w, 0x4A
	sts OCR5AL, w   
	rcall beep_beep
loop:
	rjmp loop
