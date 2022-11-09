;
; Display_Loop.asm
;

.equ B0 = (1<<0)
.equ B1 = (1<<1)
.equ B2 = (1<<2)
.equ B3 = (1<<3)
.equ B4 = (1<<4)
.equ B5 = (1<<5)
.equ B6 = (1<<6) 
.equ B7 = (1<<7) 

.equ	divx1=(0<<CS02)|(0<<CS01)|(1<<CS00)
.equ	divx8=(0<<CS02)|(1<<CS01)|(0<<CS00)
.equ	divx64=(0<<CS02)|(1<<CS01)|(1<<CS00)
.equ	divx256=(1<<CS02)|(0<<CS01)|(0<<CS00)
.equ	divx1024=(1<<CS02)|(0<<CS01)|(1<<CS00)

.equ	PRESCALER=divx256						;Timer prescaler / divider

.equ LED1 = B5
.equ S1 = B1

.equ SHIFT_CLOCK = B7	// PORT D
.equ LATCH_CLOCK = B4	// PORT D
.equ SERIAL_DATA = B0	//PORT B

.equ ELSIZE = 1

.def PortOut = r20
.def ValueIn = r16
.def DigitIn = r17

.def TimesCounter = r19
.def SerialData = r18

.def ADCRegister = r21

//----------- MACROS -----------

.macro		ldy
			ldi YL,LOW(@0)		; Load data memory address into Y
			ldi YH,HIGH(@0)
.endmacro

.macro		pushy
			push r30
			push r31
.endmacro

.macro		popy
			pop r31
			pop r30
.endmacro

	.org	0
	jmp		start			;Reset Vector  (note jmp 2 words rjmp 1 word)
	.org	PCI1addr
	jmp		PCI1_IRQSRV		;Pin Change Interrupt Request 1 Vector (ISR for Port C)
	.org	OC0Aaddr
	jmp		OC0A_IRQSRV		;TimerCounter0 Compare Match A


start:
	ldi r19, 10		//Initial set of the counter in the isr

	ldi PortOut, (SHIFT_CLOCK | LATCH_CLOCK | SERIAL_DATA)
	out DDRD, PortOut

	ldi PortOut, SERIAL_DATA
	out DDRB, PortOut

//  Interrupt Source: Hardware Timer 0 using OCR0A (Output Compare Register A) 
//  Waveform Generation Mode:Clear Timer on Compare Match (CTC)(WGM02=0,WGM01=1,WGM0=0)(Non PWM Mode)
//  COM0A1=0,COM0A0=0 Normal port operation, OC0A disconnected.
//


// 1-Timer Mode: CTC
		ldi	r20,(0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00)
		out	TCCR0A,r20

// 2-Clock select:Prescaler=256
		ldi	r20,(0<<FOC0A)|(0<<FOC0B)|(0<<WGM02)|PRESCALER
		out	TCCR0B,r20

// 3-Set OCR 
		ldi	r20,125
		out OCR0A,r20

// 4-Timer/Counter0 Output Compare Match A Interrupt Enable
		ldi	r20,(0<<OCIE0B)|(1<<OCIE0A)|(0<<TOIE0)
		sts	TIMSK0,r20

		sei			//Enable processor interrupts


//////////////////////////////////////////////
/////////////// Loop Infinito ////////////////
/////////////////////////////////////////////

apagado:
	ldi OutRegister, 0xFF
	out PORTB, OutRegister
apagadoloop:
	nop		//Para saturar menos la cpu colocamos dos nop
	nop
apagadocheck:
	call Get_Button_Status
	breq encendido
	jmp apagadoloop

encendido:
	ldi OutRegister, (~LED1)
	out PORTB, OutRegister
	rcall delay_1s

	ldi OutRegister, 0xFF
	out PORTB, OutRegister
	rcall delay_1s

	rjmp encendido


//Interrupt service routine (ISR) for pin PC1
// Timer int happens every (16MHz)^(-1)*256*125 = 62.5ns*256*125 = 64 * 125 useg = 2mseg
OC0A_IRQSRV	:
				push r16
				in r16,SREG			//Save processor status register
				push r16
				push CounterValue
				pushy

				call display_Refresh

				ldy tim_status
				ld r16, Y

check_call:		cpi r16, 0		//Checks that the timer has been set
				breq endisr

				dec r19			    // Here every 2 ms
				brne endisr

				ldi r19, 5


check_counter:	ldy timcnt
				ld CounterValue, Y
				
				cpi CounterValue, 0		//Checks that counter hasnt reached 0
				breq endisr_set

dec_counter:
				dec CounterValue
				sts timcnt, CounterValue

				jmp endisr


endisr_set:
				ldi r16, 0
				sts	tim_status, r16		//Sets status to 0, as reference that the timer has ended

endisr:		
				popy
				pop CounterValue
				pop r16
				out SREG,r16		//Restore processor status register
				pop r16
				
				reti				//Return from interrupt

// **********************************************************************************************
// timer_status: Checks status of the timer
//  
// Input arguments: none
// return: reg Z=1 if timer reached 0
// **********************************************************************************************
timer_status:
		pushy
		push r20

		ldy tim_status
		ld r20, Y

		andi r20, 0xFF		//Set Z=1 if var=0

		pop r20
		popy
		ret

// **********************************************************************************************
// start_timer: Start a timer for a time given by the input*10 ms
//  
// Input arguments: Time (*10ms, from 1-255) in r22
// ********************************************************************************************
start_timer:
		push r20

		sts timcnt, r22			//Sets timer to the desired time
		ldi r20, 1
		sts	tim_status, r20		//Sets status to on (1)

		pop r20
		ret
		
// **********************************************************************************************
// read_button: Reads button and saves it to RAM (prev_button)
// ********************************************************************************************
	.def Previous = r20
	.def Actual = r19

read_button:
				pushy
				push Previous
				push Actual

				ldy prev_button
				ld Previous, prev_button

				in Actual, PINC
				andi Actual, S1

				cp Previous, Actual
				brne set_flag					//Check if changed
				jmp endStatus


set_flag:		sts prev_button, Actual			//If changed, changes the button status

endStatus:		pop Actual
				pop Previous
				popy
				

// **********************************************************************************************
// Get_Button_Status: Returns Z=1 if button is pressed
// ********************************************************************************************
Get_Button_Status:
				pushy
				push Actual

				ldy prev_button
				ld Actual, Y
				cpi Actual, 0x02

				pop Actual
				popy


PCI1_IRQSRV:

				reti


//---------------------------------------------------------------------------

				.dseg					//Data Segment (i.e.RAM)

				.org 0x100				//Start of internal ram

prev_button:	.byte 1

button_status:	.byte 1