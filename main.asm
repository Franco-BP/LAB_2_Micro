;
; Timer1.asm
;
; Created: 10/20/2022 6:10:34 PM
; Author : jacoby
;
; Periodic interrupt Exmample



.equ	B0 = (1<<0)
.equ	B1 = (1<<1)
.equ	B2 = (1<<2)
.equ	B3 = (1<<3)
.equ	B4 = (1<<4)
.equ	B5 = (1<<5)
.equ	B6 = (1<<6)
.equ	B7 = (1<<7)

.equ	divx1=(0<<CS02)|(0<<CS01)|(1<<CS00)
.equ	divx8=(0<<CS02)|(1<<CS01)|(0<<CS00)
.equ	divx64=(0<<CS02)|(1<<CS01)|(1<<CS00)
.equ	divx256=(1<<CS02)|(0<<CS01)|(0<<CS00)
.equ	divx1024=(1<<CS02)|(0<<CS01)|(1<<CS00)

.equ	PRESCALER=divx256						;Timer prescaler / divider

.equ	SW1 = B1

		.org	0
		jmp		start			;Reset Vector  (note jmp 2 words rjmp 1 word)
		.org	PCI1addr
		jmp		PCI1_IRQSRV		;Pin Change Interrupt Request 1 Vector (ISR for Port C)
		.org	OC0Aaddr
		jmp		OC0A_IRQSRV		;TimerCounter0 Compare Match A



; Replace with your application code
start:
        ldi r23,B5
        out DDRB, r23
        out PORTB, r23

//  Interrupt Source: Hardware Timer 0 using OCR0A (Output Compare Register A) 
//  Waveform Generation Mode:Clear Timer on Compare Match (CTC)(WGM02=0,WGM01=1,WGM0=0)(Non PWM Mode)
//  COM0A1=0,COM0A0=0 Normal port operation, OC0A disconnected.
//


// 1-Timer Mode: CTC
		ldi	r20,(0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00)
		out	TCCR0A,r20

// 2-Clock select:Prescaler=1024
		ldi	r20,(0<<FOC0A)|(0<<FOC0B)|(0<<WGM02)|PRESCALER
		out	TCCR0B,r20

// 3-Set OCR 
		ldi	r20,125
		out OCR0A,r20

// 4-Timer/Counter0 Output Compare Match A Interrupt Enable
		ldi	r20,(0<<OCIE0B)|(1<<OCIE0A)|(0<<TOIE0)
		sts	TIMSK0,r20

		sei			//Enable processor interrupts


lp1:	jmp lp1	//endless loop



//Interrupt service routine (ISR) for pin PC1
// Timer int happens every (16MHz)^(-1)*256*125 = 62.5ns*256*125 = 64 * 125 useg = 2mseg
OC0A_IRQSRV	:
				in r16,SREG			//Save processor status register
				push r16

			
				dec r19			    // Here every 2 ms
				brne endisr

				ldi	r19,100			//Here every 10 x 2 ms = 20ms

toggle:			in r24,PORTB		//Toggle led 
				ldi r23,B5
				eor r24,r23
				out PORTB, r24		//PORTB->PORTB xor B5

endisr:			pop r16
				out SREG,r16		//Restore processor status register
				
				reti				//Return from interrupt

PCI1_IRQSRV:

				reti