

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

.equ	SW1 = B1

.equ SHIFT_CLOCK = B7	// PORT D
.equ LATCH_CLOCK = B4	// PORT D
.equ SERIAL_DATA = B0	//PORT B

.def ValueIn = r16
.def DigitIn = r17

.def Contador1 = r16
.def Contador2 = r17

.def SerialData = r18

.def PortOut = r20
.def ADCRegister = r21

.def TimesCounter = r23


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

.macro		loadValues
			pushy
			push ValueIn

			ldy valores_numero
			ldi ValueIn, @0
			st Y+, ValueIn

			ldi ValueIn, @1
			st Y+, ValueIn

			ldi ValueIn, @2
			st Y+, ValueIn

			ldi ValueIn, @3
			st Y+, ValueIn

			pop ValueIn
			popy
.endmacro


.equ ELSIZE = 1


	.org	0
	jmp		start			;Reset Vector  (note jmp 2 words rjmp 1 word)
	.org	PCI1addr
	jmp		PCI1_IRQSRV		;Pin Change Interrupt Request 1 Vector (ISR for Port C)
	.org	OC0Aaddr
	jmp		OC0A_IRQSRV		;TimerCounter0 Compare Match A

//----------------------------------------------------------------------------------

start:		clr r24			//Sets the timer off when starting

			ldi r20, 4
			sts digit_counter, r20		//Sets initial digit_counter value

			loadValues 0, 1, 2, 3

			ldi r19, 5		//Initial load of the register for interruption every 10ms

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


num1:		//Infinite Loop
	loadValues 0, 7, 0, 0
	ldi r22, 700/10
	call start_timer

wait1:
	nop
	nop
	call timer_status
	breq num2

	jmp wait1

num2:
	loadValues 0, 5, 0, 0
	ldi r22, 500/10
	call start_timer

wait2:
	nop
	nop
	call timer_status
	breq num1

	jmp wait2



//*********************************************
//	send_digit
//	Recibe un valor y el d??gito y lo muestra en el display. Valor 10 para limpiar el d??gito.
//	Argumentos de entrada: valor (0:9 o 10) en r16 / d??gito (1-4) en r17.
//*********************************************
send_digit:
	push PortOut

	cpi ValueIn, 11	//Control para evitar ingresos mayores a 10
	brge end
	dec DigitIn	//Tanto para la lista (.db) como para el control, nos sirve que Digit sea de 0 -> 3
	cpi DigitIn, 4
	brge end

	rcall value_to_ss	//ingreso y retorno en r16 
	rcall send_byte		//ingreso en r16

	rcall digit_to_display	//ingreso y retorno en r17
	mov r16, DigitIn
	rcall send_byte

	ldi PortOut, LATCH_CLOCK
	out PORTD, PortOut
	nop
	nop
	ldi PortOut, 0
	out PORTD, PortOut

	end:
		pop PortOut
		ret



//**********************************************************
//	value_to_ss
//	Toma un valor de ingreso y lo convierte a su valor en el display de ss. 10 para limpiar.
//	Argumento de ingreso y retorno en r16. Valores v??lidos (0:9)
//**********************************************************
value_to_ss:
	push ADCRegister
	ldi ADCRegister, 0
  
	ldi ZL, LOW(2*ss_value)
	ldi ZH, HIGH(2*ss_value)

	add ZL, ValueIn	//Agregamos el valor ingresado, para que Z apunte al valor correspondiente de la lista
	adc ZH, ADCRegister
	
	lpm ValueIn, Z
  
	pop ADCRegister
	ret

//**********************************************************
//	digit_to_display:
//	Toma un digito de ingreso y lo convierte a su valor en el display.
//	Argumento de entrada y retorno en r17. Valores v??lidos (0:3)
//**********************************************************
digit_to_display:
  push ADCRegister
  ldi ADCRegister, 0

	ldi ZL, LOW(2*display_digit_value)
	ldi ZH, HIGH(2*display_digit_value)

	add ZL, DigitIn
	adc ZH, ADCRegister
	lpm DigitIn, Z
  
  pop ADCRegister
	ret
	

//*************************************************
// send_byte
// Esta funci??n toma un byte de ingreso y lo env??a al 74HC595
// Argumento de entrada r16.
//*************************************************

send_byte:
	push TimesCounter
	push SerialData
	push PortOut
	push ADCRegister

	ldi TimesCounter, 8
	ldi ADCRegister, 0
	
	loadLoop:
		ldi SerialData, 0

		ror ValueIn
		adc SerialData, ADCRegister

		out PORTB, SerialData

		ldi PortOut, SHIFT_CLOCK
		out PORTD, PortOut
		nop		//Delay necesario para evitar fallos con la carga del dato
		nop
		ldi PortOut, 0	//(SHIFT_CLOCK xor SHIFT_CLOCK))
		out PORTD, PortOut

		dec TimesCounter
		cpi TimesCounter, 0
		brne loadLoop		//Finaliza el Loop luego de cargar el ??ltimo bit (8 veces)
	
  pop ADCRegister
  pop PortOut
	pop SerialData
	pop TimesCounter
	ret


//C??digo en hexa correspondiente al display de cada n??mero (0:9 o 10 para borrar)
ss_value:
	.db 0x03, 0x9F, 0x25, 0x0D, 0x99, 0x49, 0x41, 0x1F, 0x01, 0x19, 0xFF, 0x00	//Se agrega 0x00 para evitar padding

//C??digo en hexa correspondiente al d??gito (1:4)
display_digit_value:
	.db 0x80, 0x40, 0x20, 0x10


// ***************************************
// delay_ms
// Esta funci??n hace un delay de 2ms.
// ***************************************
delay_ms:
	push Contador1
	push Contador2
	
	ldi Contador1, 255	// 1 clk
	ldi Contador2, 40	// 1 clk
	
	loop1:
	dec Contador1		// 1 clk - Settea el flag Z si es 0
	
	brne loop1	// 1/2 clk

		ldi Contador1, 255	// 1 clk
		dec Contador2		// 1 clk - Settea el flag Z si es 0

		brne loop1	// 2 clk (-1 al final)

	pop Contador2
	pop Contador1
	ret



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
		



//Interrupt service routine (ISR) for pin PC1
// Timer int happens every (16MHz)^(-1)*256*125 = 62.5ns*256*125 = 64 * 125 useg = 2mseg
	.def CounterValue = r18

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

//--------------------------------------------------------------------------

PCI1_IRQSRV:	reti


// **********************************************************************************************
// display_Refresh: Refreshes the display with the saved values in .byte valores_numero
// ********************************************************************************************

display_Refresh:
		pushy
		push r20
		push r17
		push ADCRegister
		push DigitIn
		push ValueIn

		ldy digit_counter
		ld r20, Y

		dec r20	
		breq display_Refresh0

		cpi r20, 1
		breq display_Refresh1

		cpi r20, 2
		breq display_Refresh2

		cpi r20, 3
		breq display_Refresh3
				
		display_Refresh0:	ldi r20, 4		//When the counter gets to 0, it sets it back to 4
							ldi r17, 0
							jmp refresh

		display_Refresh1:	ldi r17, 1
							jmp refresh

		display_Refresh2:	ldi r17, 2
							jmp refresh

		display_Refresh3:	ldi r17, 3
							jmp refresh


		refresh:
		//Loads the Number to ValueIn
						clr ADCRegister
						ldy valores_numero

						add YL, r17
						adc YH, ADCRegister
						ld ValueIn, Y

		//Loads the Digit to DigitIn
						ldi ZL, LOW(valores_digito*2)
						ldi ZH, HIGH(valores_digito*2)
				
						add ZL, r17
						adc ZH, ADCRegister
						lpm DigitIn, Z
						
						call send_digit

		sts digit_counter, r20			//Saves counter status

		pop ValueIn
		pop DigitIn
		pop ADCRegister
		pop r17
		pop r20
		popy
		ret


//---------------------------------------------------------------------------

valores_digito:
	.db 1, 2, 3, 4
//---------------------------------------------------------------------------

				.dseg					//Data Segment (i.e.RAM)

				.org 0x100				//Start of internal ram

timcnt:				.byte	1*ELSIZE		//Saves byte*10ms for the timer

valores_numero:		.byte	4*ELSIZE		//Saves 1 byte per display value in the given order (1, 2, 3, 4)

digit_counter:		.byte	1				//Saves a counter as reference for the dislay digit

tim_status:			.byte	1				//Saves 0 or 1 depending on timer status
