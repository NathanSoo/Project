;
; pb0pb1led.asm
;
; Created: 11/11/2024 3:42:54 PM
; Author : 87479
;

.equ PATTERN_COUNT = 10
.def temp=r16
.def leds = r17
.def pattern_index = r18

.macro Clear
	ldi YL, low(@0)  
	ldi YH, high(@0)     
	clr temp
	st Y+, temp  
	st Y, temp                
.endmacro

.dseg
SecondCounter:
	.byte 2                   
TempCounter: 
	.byte 2
                   
.cseg
.org 0x0000 
	jmp RESET       
	jmp DEFAULT      
	jmp DEFAULT       
   
.org OVF0addr
	jmp Timer0OVF          
   
	jmp DEFAULT       
DEFAULT:  reti        


RESET:
	ser temp    ; set Port C as output
	out DDRC, temp
	clr pattern_index
	rjmp main 

Patterns:
	.db 0b11110000, 0b11001100, 0b10101010, 0b10011001  ; Pattern 0-3
    .db 0b11100111, 0b10111101, 0b10000001, 0b11111111  ; Pattern 4-7
    .db 0b01111110, 0b00011000                          ; Pattern 8-9


Timer0OVF:
    ; Prologue
    push temp
    in temp, SREG
    push temp
    push YH
    push YL
    push r24
    push r25

    ; Load TempCounter and increment by 1
    ldi YL, low(TempCounter)
    ldi YH, high(TempCounter)
    ld r24, Y+
    ld r25, Y
    adiw r25:r24, 1

    ; Check if TempCounter has reached 2000 (2 seconds)
    cpi r24, low(2000)
    brne NotSecond
    cpi r25, high(2000)
    brne NotSecond

    ; Toggle LEDs by cycling through patterns
    ldi ZL, low(Patterns*2)
    ldi ZH, high(Patterns*2)
    add ZL, pattern_index       ; Use pattern_index to select pattern
    lpm leds, Z                 ; Load pattern from program memory into leds
    out PORTC, leds             ; Output to Port C

    ; Reset TempCounter
    Clear TempCounter

    ; Increment pattern_index and reset if it reaches PATTERN_COUNT
    inc pattern_index
    cpi pattern_index, PATTERN_COUNT
    brne Continue
    clr pattern_index           ; Reset pattern_index to 0 if reached 10

Continue:           
NotSecond:
    st Y, r25                ; Store the value of the temporary counter.
    st -Y, r24
	pop r24                ; Epilogue starts; 
	pop r25                ; Restore all conflict registers from the stack. 
	pop YL
	pop YH
	pop temp
	out SREG, temp
	pop temp
	reti                       ; Return from the interrupt.
      ; continued
main: 
	ldi leds, 0xff  
	out PORTC, leds
	ldi leds, 0b11110000
	Clear TempCounter                
	Clear SecondCounter             
	ldi temp, 0b00000000
	out TCCR0A, temp
	ldi temp, 0b00000011
	out TCCR0B, temp                    
	ldi temp,  1<<TOIE0                
	sts TIMSK0, temp                  
sei                                                 
loop:  
rjmp loop                 
