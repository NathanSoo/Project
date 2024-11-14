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
TempCounter:	  .byte 2 ; Temporary counter used to determine if 1 second has passed
SecondCounter:    .byte 1 ; Counter to store seconds
BeepCounter:      .byte 1 ; Counter for keeping track of times beeped
BeepState:		  .byte 1 ; Maintains state for beep routines
BeepSeconds:      .byte 1 ; Stores number of seconds for motor to beep

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
	push r24						
	push r19						; Prologue ends
	ldi  YL, low(TempCounter) 
	ldi  YH, high(TempCounter)		; Y pointer points to temp counter
	ld   r24, Y+					; Load the value of the temporary counter.
	ld   r25, Y
	adiw r25:r24, 1					; Increase the temporary counter by one.
	cpi  r24, low(1000)			; Check if (r25:r24)=1000
	brne not_second
	cpi  r25, high(1000)
	brne not_second
	clear_word TempCounter
	ldi  YL, low(SecondCounter)		; Load the address of the second counter.
	ldi  YH, high(SecondCounter)		
	ld   r24, Y+					; Load the value of the second counter.
	ld   r25, Y
	adiw r25:r24, 1					; Increase the second counter by one.
	st   Y, r25						; Store the value of the second counter.
	st   -Y, r24
	mov  r19, r24					; Save value of second counter
	ldi  YH, high(BeepState)
	ldi  YL, low(BeepState)
	ld	 r24, Y
	cpi  r24, 1
	breq do_beep_beep
do_beep_beep:
	ldi YH, high(BeepSeconds)
	ldi YL, low(BeepSeconds)
	ld  w, Y
	cp  r19, w
	breq end_beep_beep
	ldi w, 0b00001000				; Toggle motor on/off
	eor motor_state, w
	sts DDRL, motor_state
	rjmp endif
end_beep_beep:
	rcall init_beeps
	rjmp endif
not_second:
	st   Y, r25
	st  -Y, r24
endif:
	pop  r19
	pop  r24
	pop  r25
	pop  YL
	pop  YH
	pop  w
	out  SREG, w
	pop  w
	reti

; Play beep_beep tone for 10 seconds
beep_beep:
	push YH
	push YL
	push w
	ldi  YH, high(BeepState)
	ldi  YL, low(BeepState)
	ldi  w, 1                       ; Store 1 for beep beep mode
	st   Y, w
	ldi  YH, high(BeepSeconds)
	ldi  YL, low(BeepSeconds)
	ldi  w, 10                      ; Number of seconds to beep for
	st   Y, w
	ldi  w, 1<<TOIE0				; Enable timer 0 overflow interrupt
	sts  TIMSK0, w
	ldi  w, 0b00001000
	sts  DDRL, w					; Bit 3 will function as OC5A. (Motor on)
	pop  w
	pop  YL
	pop  YH
	ret

init_beeps:
	push YH
	push YL
	push w
	ldi  YH, high(BeepState)
	ldi  YL, low(BeepState)
	clr  w
	st   Y, w						; Set beep state to 0
	clr  w							; Disable timer 0 overflow interrupt
	sts  TIMSK0, w
	clear_word TempCounter
	clr  w							; Motor off
	sts  DDRL, w
	clr  motor_state
	pop  w
	pop  YL
	pop  YH
	ret

reset:
	rcall init_beeps
	; Set Timer 0 to count seconds
	clr w
	out TCCR0A, w				    ; Set Timer 0 to Normal Mode
	ldi w, 0b00000011
	out TCCR0B, w					; Prescaler value = 64	
	sei								; Enable global interrupt
	; Set Timer 5 for waveform generation
	clr w						; the value controls the PWM duty cycle
	sts OCR5AH, w				; Set higher byte of OCR5A to 0
	ldi w, 0x4A        
	sts OCR5AL, w				; Set lower byte to 0x4A
	; Set Timer5 to Phase Correct PWM mode.
	ldi w, 0b00001001			; Set Timer clock frequency (no prescaling)
	sts TCCR5B, w      
	ldi w, (1<< WGM50)|(1<<COM5A1) ; Phase correct PWM mode
	sts TCCR5A, w
	rcall beep_beep
loop:
	rjmp loop
