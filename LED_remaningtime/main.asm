;
; LED2.asm
;
; Created: 13/11/2024 3:11:52 PM
; Author : 87479
;


.include "m2560def.inc"
.def pattern_index = r18
.equ PATTERN_COUNT = 8

 .equ PATTERN=0b11110000
 .def temp=r16
 .def leds = r17
 ; The macro clears a word (2 bytes) in the data memory for the counter stored in the memory
 ; The parameter @0 is the memory address for that word 
.macro Clear
 ldi YL, low(@0)  
; load the memory address to Y 
ldi YH, high(@0)     
clr temp
 st Y+, temp  
st Y, temp                
.endmacro
 ; clear the two bytes at @0 in SRAM
 ; continued
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
;…   
.org OVF0addr
 jmp Timer0OVF          
;…   
jmp DEFAULT       
DEFAULT:  reti        
; Two-byte counter for counting the number of seconds.   
; Temporary counter. Used to determine 
; if one second has passed (i.e. when TempCounter=1000)   
; No handling for IRQ0.
 ; No handling for IRQ1.
 ; insert other interrupt vectors
 ; Jump to the interrupt handler for Timer0 overflow. 
; other default service
 ; default service for all other interrupts.   
; no service
 ; continued
PatternsE:
	.db 0b01111111, 0b00111111, 0b00011111, 0b00001111 ; Pattern 1-3
    .db  0b00000111, 0b00000011, 0b00000001 ,0b00000000 ; Pattern 4-7
     
  RESET:
                      
              ser temp    ; set Port C as output
 sts DDRk, temp
 out DDRg, temp
 
 rjmp main

  Timer0OVF:   ; interrupt subroutine for Timer0
               push temp           ; Prologue starts.
               in temp, SREG
               push temp           ; Prologue starts.
               push Yh               ; Save all conflict registers in the prologue. 
               push YL 
               push r25  
               push r24               ; Prologue ends.
               ldi YL, low(TempCounter)    ; Load the address of  the temporary
               ldi YH, high(TempCounter)   ; counter.
               ld r24, Y+             ; Load the value of the temporary counter.
               ld r25, Y
               adiw r25:r24, 1    ; Increase the temporary counter by one
 cpi r24, low(2000)            ; Check if (r25:r24)=1000  
              brne NotSecond
              cpi  r25, high(2000)        
              brne  NotSecond
              
      ;com leds       
              Clear TempCounter  ; Reset the temporary counter.
               ldi YL, low(SecondCounter)      ; Load the address of  the second
               ldi YH, high(SecondCounter)    ; counter.
               ld r24, Y+              ; Load the value of the second counter.
               ld r25, Y    
			   
			  cpi r24, 8
			  breq led_1
			  cpi r24, 9
			  breq led_2

			   ldi ZL, low(PatternsE*2)
			ldi ZH, high(PatternsE*2)
			add ZL, r24
			lpm leds, Z
			sts PORTK, leds 
			rjmp end_leds
			
led_1:		
			ldi leds, 0b00000001
			out PORTG, leds
			rjmp end_leds

			
led_2:		
			ldi leds, 0b000000000
			out PORTG, leds
			rjmp end_leds
		   
end_leds:          
               adiw r25:r24, 1     ; Increase the second counter by one.

			   cpi r24, 10
			   brne store_second
			   clr r24
			   sts TCCR1A, r24
			   sts TCCR1B, r24
			   sts TCNT1h, r24
			   sts TCNT1l, r24
			   sts TIMSK1, r24
store_second:
			   st Y, r25                ; Store the value of the second counter.
               st -Y, r24 
               rjmp endif              
NotSecond:
               st Y, r25                ; Store the value of the temporary counter.
               st -Y, r24
 endif:
               pop r24                ; Epilogue starts; 
               pop r25                ; Restore all conflict registers from the stack. 
               pop YL
               pop YH
               pop temp
               out SREG, temp
               pop temp
               reti                       ; Return from the interrupt
 main: 
ldi leds, 0xff  
sts PORTK, leds
out PORTG, leds
 ldi leds, PATTERN
 Clear TempCounter                
Clear SecondCounter             
ldi temp, 0b00000000
 sts TCCR1A, temp
 ldi temp, 0b00000011
 sts TCCR1B, temp                    
ldi temp,  1<<TOIE0                
sts TIMSK1, temp                  
sei                                                 
loop:  
rjmp loop
