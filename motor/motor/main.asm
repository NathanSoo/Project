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
wCounter:	  .byte 2 ; worary counter used to determine if 1 second has passed

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
	ldi  YL, low(wCounter) 
	ldi  YH, high(wCounter)		; Y pointer points to w counter
	ld   r24, Y+					; Load the value of the worary counter.
	ld   r25, Y
	adiw r25:r24, 1					; Increase the worary counter by one.
	cpi  r24, low(1000)				; Check if (r25:r24)=1000
	brne not_second
	cpi  r25, high(1000)
	brne not_second

	ldi w, 0b00000001				; Set Timer clock off
	eor motor_state, w
	sts TCCR5B, w
	ldi w, 0
	out PORTL, 0

	; set timer scaling off
	; set output pin to 0
	clear_word wCounter
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
	ldi w, 1<<TOIE0				; Enable timer overflow interrupt
	sts TIMSK0, w	
	sei								; Enable global interrupt
	; Set Timer 5 for waveform generation
	ldi w, 0b00001000
	out DDRL, w					; Bit 3 will function as OC5A.
	clr w						; the value controls the PWM duty cycle
	sts OCR5AH, w				; Set higher byte of OCR5A to 0
	ldi w, 0x4A        
	sts OCR5AL, w				; Set lower byte to 0x4A
	; Set Timer5 to Phase Correct PWM mode.
	ldi w, 0b00001001			; Set Timer clock frequency (no prescaling)
	sts TCCR5B, w      
	ldi w, (1<< WGM50)|(1<<COM5A1) ; Phase correct PWM mode
	sts TCCR5A, w
	ldi motor_state, 1			; Init motor state to 1
loop:
	rjmp loop
