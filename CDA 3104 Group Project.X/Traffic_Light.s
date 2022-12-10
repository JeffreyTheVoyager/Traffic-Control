; File: Traffic_Light.s
; Class: CDA 3104, Fall 2022
; Devs: Jeffrey Abraham, Justin Rubio, Billy Jean Baptiste        
; Description: Traffic Light System

#define __SFR_OFFSET 0
#include <avr/io.h>

; vector table for interrupts
;----------------------------------------------------------
BEGIN_VECTORS:                          ; Reset
          jmp       setup
INT0addr:                               ; External Interrupt Request 0
          jmp       button_isr
INT1addr:                               ; External Interrupt Request 1
          nop       
          nop
PCI0addr:                               ; Pin Change Interrupt Request 0
          nop       
          nop
PCI1addr:                               ; Pin Change Interrupt Request 0
          nop       
          nop
PCI2addr:                               ; Pin Change Interrupt Request 1
          nop       
          nop
WDTaddr:                                ; Watchdog Time-out Interrupt
          nop       
          nop
OC2Aaddr:                               ; Timer/Counter2 Compare Match A
          nop       
          nop
OC2Baddr:                               ; Timer/Counter2 Compare Match A
          nop       
          nop
OVF2addr:                               ; Timer/Counter2 Overflow
          nop       
          nop
ICP1addr:                               ; Timer/Counter1 Capture Event
          nop       
          nop
OC1Aaddr:                               ; Timer/Counter1 Compare Match A
          jmp       timer_isr
OC1Baddr:                               ; Timer/Counter1 Compare Match B
          nop       
          nop
OVF1addr:                               ; Timer/Counter1 Overflow
          nop       
          nop
OC0Aaddr:                               ; TimerCounter0 Compare Match A
          nop       
          nop
OC0Baddr:                               ; TimerCounter0 Compare Match B
          nop       
          nop
OVF0addr:                               ; Timer/Couner0 Overflow
          nop       
          nop
SPIaddr:                                ; SPI Serial Transfer Complete
          nop       
          nop
URXCaddr:                               ; USART Rx Complete
          nop       
          nop
UDREaddr:                               ; USART, Data Register Empty
          nop       
          nop
UTXCaddr:                               ; USART Tx Complete
          nop       
          nop
ADCCaddr:                               ; ADC Conversion Complete
          nop       
          nop
ERDYaddr:                               ; EEPROM Ready
          nop       
          nop
ACIaddr:                                ; Analog Comparator
          nop       
          nop
TWIaddr:                                ; Two-wire Serial Interface
          nop       
          nop
SPMRaddr:                               ; Store Program Memory Read
          nop       
          nop
END_VECTORS:
.set BTN_INT_DIR,DDRD                  ; set pin mode in/out
.set BTN_INT_OUT, PORTD                 ; set pin in-mode hi/pu or out-state 
;high/low 
.set BTN_INT_IN,  PIND                  ; read pin-state high/low
.set BTN_INT_PIN, 2                     ; (D2) DDD2/PORTD2/PIND2
.set BTN_LED_DIR, DDRB                  ; set pin mode in/out
.set BTN_LED_OUT, PORTB                 ; set in mode hi/pu or out high/low 
.set BTN_LED_IN,  PINB                  ; read pins high/low
.set BTN_LED_PIN, 1                     ; (D8) DDB0/PORTB0
.set TMR_LED_DIR, DDRB                  ; set pin mode in/out
.set TMR_LED_OUT, PORTB                 ; set in mode hi/pu or out high/low 
.set TMR_LED_IN,  PINB                  ; read pins high/low
.set TMR_LED_PIN, 3                     ; (D11) DDB3/PORTB3
; 1-time Confgiure devices
;----------------------------------------------------------
setup:
          ; button LED pin for output and off
          sbi       BTN_LED_DIR, BTN_LESD_PIN      
          cbi       BTN_LED_OUT, BTN_LED_PIN      
          ; timer LED pin for output and off
          sbi       TMR_LED_DIR, TMR_LED_PIN
          cbi       TMR_LED_OUT, TMR_LED_PIN
          ; button on ext INT0 input and pull-up
          cbi       BTN_INT_DIR, BTN_INT_PIN
          sbi       BTN_INT_OUT, BTN_INT_PIN
          ; configure button interrupt
          sbi       EIMSK,INT0          ; enable INT0 on D2 for button
          ldi       r20,0b00000010      ; 
          sts       EICRA,r20           ; set falling edge
          ; configure Timer1
          clr       r20                 ; set zero in Timer 1
          sts       TCNT1H,r20          ; counter
          sts       TCNT1L,r20
          ; put 15624 (250ms w/ 256 scaler)
          ; in Output Compare Match Registers
          ldi       r20,0x3D            ; high byte of 250ms
          sts       OCR1AH,r20          ; OCR1A High Byte
          ldi       r20,0x08            ; low byte of 250ms
          sts       OCR1AL,r20          ; OCR1A Low Byte
          ; setting CTC mode and clk select 256
          clr       r20                 
          sts       TCCR1A,r20          ; clear WGM11 and WGM10
          ldi       r20,0b00001100      ; set WGM12 and CS12
          sts       TCCR1B,r20          ; START
          ldi       r20,0b00000010      ; enable OCIE1A interrupt
          sts       TIMSK1,r20          ; for Timer1
          sei                           ; enable global interrupts
                                        ; end_setup
; endless loop for main application
;----------------------------------------------------------
loop:
          rjmp      loop                ; end_loop
; button press interrupt service routine
;----------------------------------------------------------
button_isr:
          ; see if button LED is high/low
          sbis      BTN_LED_IN, BTN_LED_PIN
          rjmp      btn_turn_on
          rjmp      btn_turn_off
btn_turn_on:
          ; turn button LED on
          sbi       BTN_LED_OUT, BTN_LED_PIN        
          rjmp      end_button_isr
btn_turn_off:
          ; turn button LED off
          cbi       BTN_LED_OUT, BTN_LED_PIN        
end_button_isr:
          reti                          ; end button_isr
; timer interrupt service routine
;----------------------------------------------------------
timer_isr:
          ; see if timer LED is high/low
          sbis      TMR_LED_IN, TMR_LED_PIN
          rjmp      tmr_turn_on
          rjmp      tmr_turn_off
tmr_turn_on:
          ; turn timer LED on
          sbi       TMR_LED_OUT, TMR_LED_PIN        
          rjmp      end_timer_isr
tmr_turn_off:
          ; turn timer LED off
          cbi       TMR_LED_OUT, TMR_LED_PIN        
end_timer_isr:
          reti                          ;end timer_isr          
          
start:

          /*NORTH SOUTH TRAFFIC LIGHT*/
          /*SET TO OUTPUT*/
          SBI       DDRD,DDD0           ;GREEN
          SBI       DDRD,DDD1           ;YELLOW
          SBI       DDRD,DDD2           ;RED
          
                    
          /*NORTH SOUTH CROSS WALK LIGHT*/
          SBI       DDRD,DDD3           ;RED           
          SBI       DDRD,DDD4           ;WHITE
          
          rcall      turn_north_south_led_off
          
          
          /*EAST WEST TRAFFIC LIGHT*/
          /*SET TO OUTPUT*/
          SBI       DDRB,DDB0           ;GREEN
          SBI       DDRB,DDB1           ;YELLOW
          SBI       DDRB,DDB2           ;RED
          

          /*EAST WEST CROSS WALK LIGHT*/
          SBI       DDRB,DDB3           ;RED
          SBI       DDRB,DDB4           ;WHITE
          
          rcall      turn_east_west_led_off
         

          CBI       DDRB, DDB5          ;PHOTO RESISTOR
          
          CBI       DDRB,DDB6           ;ARCOSS NORTH SOUTH LANE BUTTON
          
          CBI       DDRB,DDB7           ;ACROSS EAST WEST LANE BUTTON
          
          
main:
          
          
          /*ENABLE PULL-UP BUTTON*/
          SBI       PORTD,PORTD7        
          SBI       PORTD,PORTD6        
          SBI       PORTD,PORTD3
          SBI       PORTD,PORTD3        
          
          
          /*NORTH SOUTH TRAFFIC FLOW*/
          call      traffic_flow_north_south
          
          SBIS      PINB,PINB4          ;SKIP IF DAY
          call      night_flow_delay
          
          SBI       PORTD,PORTD1        ;NORTH SOUTH YELLOW LIGHT ON
          
          call      delay
          call      delay
          call      delay
          
    
          /*SWITCH NORTH SOUTH FLOW*/
          CBI       PORTD,PORTD1        ;NORTH SOUTH YELLOW LIGHT OFF
          
          call      delay
          call      delay
          call      delay
          
          SBI       PORTD,PORTD2        ;NORTH SOUTH RED LIGHT ON
          
          
          /*EAST WEST TRAFFIC FLOW*/
          call      traffic_flow_east_west
          
          SBIC      PINB,PINB4          ;SKIP IF DAY
          call      night_flow_delay
          
          
          /*SWITCH EAST WEST FLOW*/
          CBI       PORTB,PORTB1        ;EAST WEST YELLOW LIGHT OFF
          
          call      delay
          call      delay
          call      delay
          
          SBI       PORTB,PORTB2        ;EAST WEST RED LIGHT ON
          
          rjmp      main
          
          
traffic_flow_north_south:
          
          SBI       PORTD,PORTD0        ;NORTH SOUTH GREEN LIGHT ON
          SBI       PORTB,PORTB2        ;EAST WEST RED LIGHT ON
          
          SBI       PORTD,PORTD3        ;NORTH SOUTH CROSSING RED LIGHT ON
          
          call      delay
          call      delay
          call      delay
          
          CBI       PORTD,PORTD0        ;NORTH SOUTH GREEN LIGHT OFF
          
          
traffic_flow_east_west:
          
          SBI       PORTB,PORTB0        ;EAST WEST GREEN LIGHT ON
          SBI       PORTD,PORTD2        ;NORTH SOUTH RED LIGHT ON
          
          SBI       PORTB,PORTB3        ;EAST WEST CROSSING RED LIGHT ON
          
          call      delay
          call      delay
          call      delay
          
          CBI       PORTB,PORTB2        ;EAST WEST RED LIGHT ON
          
 
night_flow_delay:
          
          call      delay
          call      delay
          call      delay
          
          
turn_north_south_led_off:
          
          CBI       PORTD,PORTD0
          CBI       PORTD,PORTD1
          CBI       PORTD,PORTD2
          
          /*CROSSING LIGHT*/
          CBI       PORTD,PORTD3
          CBI       PORTD,PORTD4
          
          call      delay
          
          
turn_east_west_led_off:
          
          CBI       PORTB,PORTB0
          CBI       PORTB,PORTB1
          CBI       PORTB,PORTB2
          
          /*CROSSING LIGHT*/
          CBI       PORTB,PORTB3
          CBI       PORTB,PORTB4
          
          call      delay

north_south_cross_walk:
          
          call      turn_north_south_led_off
          
          SBI       PORTD,PORTD3        ;NORTH SOUTH RED LIGHT ON
          
          CBI       PORTD,PORTD3        ;NORTH SOUTH CROSSING RED LIGHT OFF                 
          SBI       PORTD,PORTD4        ;NORTH SOUTH CORSSING WHITE LIGHT ON
          
          call      delay
          call      delay
          call      delay
          
delay:
          LDI       r20,24
          
wait_1:
          LDI       r21,100
wait_2:          
          LDI       r22,150
wait_3:          
          NOP
          NOP
          DEC       r22
          BRNE      wait_3
          
          DEC       r21
          BRNE      wait_2
          
          DEC       r20
          BRNE      wait_1
          RET
                    


