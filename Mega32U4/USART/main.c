
;***********************************************************
;*
;*	This is the TRANSMIT skeleton file for Lab 7 of ECE 375
;*
;*  	Rock Paper Scissors
;* 	Requirements:
;* 	1. USART1 communication
;* 	2. Timer/counter1 Normal mode to create a 1.5-sec delay
;***********************************************************
;*
;*	 Author: Christopher Dueber
;*	   Date: 11/25/23
;*
;***********************************************************

.include "m32U4def.inc"         ; Include definition file

;***********************************************************
;*  Internal Register Definitions and Constants
;***********************************************************
.def    mpr = r16                ; Multi-Purpose Register
.def	waitcnt = r17			 ; Wait Loop Counter
.def    ilcnt = r18              ; Inner Loop Counter
.def    olcnt = r19              ; Outer Loop Counter
.def	GCount = r23
.def	RXReg = r24
.def	TXReg = r25

; Use this signal code between two boards for their game ready
.equ    SendReady = 0b11111111

.equ	WTime = 50									; 150 ms MAX wait

.equ	Gestures = 2
.equ	StringCount = 16


;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg												; Beginning of code segment

;***********************************************************
;*  Interrupt Vectors
;***********************************************************
.org    $0000										; Beginning of IVs
	    rjmp    INIT            					; Reset interrupt

.org	$0002	
		rcall	GESTURE								; Iterrate through gestures (wrapped)
		reti										; Rock -> Paper -> Scissor

.org    $0004
        rcall   GAME_BEGIN							; Wait for opponent, Tx/Rx comms
        reti

.org	$0032
		rcall	USART_Receive						; USART1 Rx Complete interrupt
		reti

.org    $0056										; End of Interrupt Vectors

;***********************************************************
;*  Program Initialization
;***********************************************************
INIT:
		;Stack Pointer (VERY IMPORTANT!!!!)
		; Initialize Stack Pointer
		ldi		mpr, low(RAMEND)
		out		SPL, mpr							; Load SPL with low byte of RAMEND
		ldi		mpr, high(RAMEND)
		out		SPH, mpr							; Load SPH with high byte of RAMEND

		;I/O Ports
		; Initialize Port B for output
		ldi		mpr, $FF							; Set Port B Data Direction Register
		out		DDRB, mpr							; for output
		ldi		mpr, $00							; Initialize Port B Data Register
		out		PORTB, mpr							; so all Port B outputs are low

		; Initialize Port D for external interrupts
		; Transmitter: 
		; RXD1[PD3]: Receive input [0]
		; TXD1[PD2]: Send output [1]
		ldi		mpr, 0b00001000						; Set Port D Data Direction Register
		out		DDRD, mpr							; for input
		ldi		mpr, $FF
		out		PORTD, mpr


		; USART1 setup
		ldi		mpr, high(416)						; UBRR value for 2400 bps baudrate
		sts		UBRR1H, mpr
		ldi		mpr, low(416)						; 8 MHz, double speed mode
		sts		UBRR1L, mpr

		; Set frame format: 8 data bits, 2 stop bits
		ldi		mpr, (1 << USBS1) | (1 << UCSZ11) | (1 << UCSZ10) 
		sts		UCSR1C, mpr

		ldi		mpr, (1 << UDRE1) | (1 << U2X1)					; Double USART Transmission speed
		sts		UCSR1A, mpr

		;  CODE DIFFERENCE HERE - SET ONE BOARD TO RECIEVE ONE TO TRANSMIT
		ldi		mpr, (1<<TXEN1) | (1<<RXEN1) | (1<<RXCIE1)			; Enable transmit 
		sts		UCSR1B, mpr

		;TIMER/COUNTER1
		;Set Normal mode
		ldi		mpr, 0b00000000						; (WGMn3:0 = 0)
		sts		TCCR1A, mpr

		ldi		mpr, 0b00000011						; 64 prescaler
		sts		TCCR1B, mpr	

	    ; Configure External Interrupts
        ; Falling edge
		ldi		mpr, 0b00001010						; set falling edge interrupts: INT1, INT0
		sts		EICRA, mpr							; stores EICRA

		; Configure the External Interrupt Mask
		ldi		mpr, 0b0000011						; Allows interrupt flags
		out		EIMSK, mpr							; stores EIMSK

		rcall	LCDInit								; routine call to initialize the LCD
		rcall	LCDBacklightOn
		rcall	LCDClr								; clear the LCD

		ldi		GCount, Gestures
		rcall	LOAD_WELCOME

		sei


;***********************************************************
;*  Main Program
;***********************************************************
MAIN:
		


		rjmp	MAIN


;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func:	USART_Transmit
; Desc:	Transmit data to receiver, polling UDRE1 flag
;-----------------------------------------------------------

USART_Transmit:
		lds     mpr, UCSR1A
		sbrs	mpr, UDRE1
		rjmp	USART_Transmit

		sts		UDR1, TXReg

		ret
;-----------------------------------------------------------
; Func:	USART_Receive
; Desc:	Transmit data to receiver, polling UDRE1 flag
;-----------------------------------------------------------

USART_Receive:		
		; DO SOMETHING
		lds		RXReg, UDR1

		ret


;-----------------------------------------------------------
; Func:	GAME_BEGIN
; Desc:	Interrupt function, PD7 Button press and transmit
; ready signal to receiver.
;-----------------------------------------------------------
GAME_BEGIN:
		ldi		TXReg, SendReady			; Test message
		rcall	LOAD_WAIT					; Load wait for ready signal 
		rcall	USART_Transmit				; Transmit ready signal

RX_COMPLETE:
		cpi		RXReg, SendReady			; Compare Ready signal
		brne	RX_COMPLETE

		rcall	COUNTDOWN					; To show receive completed
		
		ret


;-----------------------------------------------------------
; Func:	DELAY
; Desc:	Used primarily for COUNTDOWN function. Calls a delay
; of 1.5 seconds by looping a 500ms delay three times.
;-----------------------------------------------------------
DELAY:
		push	waitcnt

		ldi		waitcnt, 3					; Set loop counter

CHECKFLAG:
		sbis	TIFR1, TOV1					; Check if TOV1 flag has been set
		rjmp	CHECKFLAG					; Skip CHECKFLAG if Bit 0 is set to 1

		sbi		TIFR1, TOV1					; No Overflow Interrupt, Set TOV1 to 1 to clear
									
		ldi		mpr, 0xDC					; Counter value, set flag as TCNT1 becomes zero
		sts		TCNT1L, mpr					; TCNT1x = 
		ldi		mpr, 0x0B					; prescaler/F_Clock = 64/8 Mhz = 0.000008
		sts		TCNT1H, mpr					; (500 ms) / 0.000008 = 62500
											; 2^16 - 62500 = 3036 -> $0BDC

		dec		waitcnt						; Decrement waitcnt by 1 for each loop
		cpi		waitcnt, 0			
		brne	CHECKFLAG

		pop		waitcnt

		ret

;-----------------------------------------------------------
; Func:	SPLASH SCREEN
; Desc:	Prints first screen to LCD 
;-----------------------------------------------------------

LOAD_WELCOME:
		ldi		ZH, high(WELCOME<<1)		; Loads HIGH byte Z, shift
		ldi		ZL, low(WELCOME<<1)			; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr)	; Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr)	; Loads LOW byte Y, driver defined address 

		ldi		waitcnt, stringCount		; Loads string count into counter register

WLOOP:	
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	WLOOP
		
		rcall	LCDWrLn1

		ldi		ZH, high(PRESS_START<<1)	; Loads HIGH byte Z, shift
		ldi		ZL, low(PRESS_START<<1)		; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr+16); Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr+16)	; Loads LOW byte Y, driver defined address
WLOOP2:
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	WLOOP2

		rcall	LCDWrLn2

		ret

;-----------------------------------------------------------
; Func:	LOAD_WAIT
; Desc:	Waiting for oppenent screen, print LCD function
;-----------------------------------------------------------

LOAD_WAIT:
		ldi		ZH, high(WAIT1_START<<1)	; Loads HIGH byte Z, shift
		ldi		ZL, low(WAIT1_START<<1)		; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr)	; Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr)	; Loads LOW byte Y, driver defined address 

		ldi		waitcnt, stringCount		; Loads string count into counter register

WTLOOP:	
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	WTLOOP
		
		rcall	LCDWrLn1

		ldi		ZH, high(WAIT2_START<<1)	; Loads HIGH byte Z, shift
		ldi		ZL, low(WAIT2_START<<1)		; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr+16); Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr+16)	; Loads LOW byte Y, driver defined address

WTLOOP2:
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	WTLOOP2

		rcall	LCDWrLn2

		ret

;-----------------------------------------------------------
; Func:	COUNTDOWN
; Desc:	PB7-4 control and DELAY calls (1.5s x 4 = 6s)
;-----------------------------------------------------------

COUNTDOWN:
		push	mpr				; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG		; Save program state
		push	mpr				;

		sei
		
		ldi		mpr, (1 << PB7) | (1 << PB6) | (1 << PB5) | (1 << PB4)  ; Set PB7-4, leave PB3-0 unchanged
		out		PORTB, mpr
		
		rcall	DELAY

		ldi		mpr, (1 << PB6) | (1 << PB5) | (1 << PB4)  ; Set PB7-4, leave PB3-0 unchanged
		out		PORTB, mpr	

		rcall	DELAY

		ldi		mpr, (1 << PB5) | (1 << PB4)  ; Set PB7-4, leave PB3-0 unchanged
		out		PORTB, mpr

		rcall	DELAY

		ldi		mpr, (1 << PB4)  ; Set PB7-4, leave PB3-0 unchanged
		out		PORTB, mpr	

		rcall	DELAY

		ldi		mpr, (0 << PB7) | (0 << PB6) | (0 << PB5) | (0 << PB4)
		out		PORTB, mpr

		pop		mpr				; Restore program state
		out		SREG, mpr		;
		pop		waitcnt			; Restore wait register
		pop		mpr				; Restore mpr

		ret



;-----------------------------------------------------------
; Func:	GESTURE
; Desc:	Interrupt based function. Selects gesture for player 
; and iterates through gestures.
;-----------------------------------------------------------
GESTURE:
		push	mpr				; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG		; Save program state
		push	mpr				;

		cpi		GCount, 2
		breq	LOAD_ROCK		; Load Rock
				
		cpi		GCount, 1
		breq	LOAD_PAPER		; Load Paper
				
		cpi		GCount, 0
		breq	LOAD_SCISSOR	; Load Scissor
					
LOAD_ROCK:
		ldi		ZH, high(GAME_START<<1)	; Loads HIGH byte Z, shift
		ldi		ZL, low(GAME_START<<1)	; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr)	; Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr)	; Loads LOW byte Y, driver defined address 

		ldi		waitcnt, stringCount		; Loads string count into counter register

RLOOP:	
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	RLOOP
		
		rcall	LCDWrLn1

		ldi		ZH, high(GESTURE_ROCK<<1)		; Loads HIGH byte Z, shift
		ldi		ZL, low(GESTURE_ROCK<<1)		; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr+16); Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr+16)	; Loads LOW byte Y, driver defined address
RLOOP2:
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	RLOOP2

		rcall	LCDWrLn2

		cpi		GCount, 0
		brne	GDEC
		ldi		GCount, 2
		jmp		END

LOAD_PAPER:
		ldi		ZH, high(GAME_START<<1)		; Loads HIGH byte Z, shift
		ldi		ZL, low(GAME_START<<1)		; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr)	; Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr)	; Loads LOW byte Y, driver defined address 

		ldi		waitcnt, stringCount		; Loads string count into counter register

PLOOP:	
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	PLOOP
		
		rcall	LCDWrLn1

		ldi		ZH, high(GESTURE_PAPER<<1)		; Loads HIGH byte Z, shift
		ldi		ZL, low(GESTURE_PAPER<<1)		; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr+16); Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr+16)	; Loads LOW byte Y, driver defined address
PLOOP2:
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	PLOOP2

		rcall	LCDWrLn2

		cpi		GCount, 0
		brne	GDEC
		ldi		GCount, 2
		jmp		END

LOAD_SCISSOR:
		ldi		ZH, high(GAME_START<<1)		; Loads HIGH byte Z, shift
		ldi		ZL, low(GAME_START<<1)		; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr)	; Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr)	; Loads LOW byte Y, driver defined address 

		ldi		waitcnt, stringCount		; Loads string count into counter register

SLOOP:	
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	SLOOP
		
		rcall	LCDWrLn1

		ldi		ZH, high(GESTURE_SCISSOR<<1); Loads HIGH byte Z, shift
		ldi		ZL, low(GESTURE_SCISSOR<<1)	; Loads LOW byte Z, shift
		ldi		YH, high(lcd_buffer_addr+16); Loads HIGH byte Y, driver defined address
		ldi		YL, low(lcd_buffer_addr+16)	; Loads LOW byte Y, driver defined address
SLOOP2:
		lpm		mpr, Z+						; Load program memory to register, Z post-increment
		st		Y+, mpr						; Store byte to data space, Y post-increment
		dec		waitcnt
		brne	SLOOP2

		rcall	LCDWrLn2

		cpi		GCount, 0					; Wrap selection Scissor -> Rock
		brne	GDEC
		ldi		GCount, 2
		jmp		END							; Error here
											; I need to grab selection and send it to RXReg

GDEC:
		dec		GCount
		rcall	CLRQ

END:
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr

		
		ret							; End a function with RET



	

;-----------------------------------------------------------
; Func:	CLRQ
; Desc:	Clear interrupt queue
;-----------------------------------------------------------
CLRQ:	

		ldi		mpr, $FF			; Clear interrupt queue on all pins
		out		EIFR, mpr		

		ret							; End a function with RET


;----------------------------------------------------------------
; Sub:	WAIT
; Desc:	A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;		waitcnt*10ms.  Just initialize wait for the specific amount 
;		of time in 10ms intervals. Here is the general eqaution
;		for the number of clock cycles in the wait loop:
;			((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;----------------------------------------------------------------

WAIT:
		push	waitcnt				; Save wait register
		push	ilcnt				; Save ilcnt register
		push	olcnt				; Save olcnt register

Loop:	ldi		olcnt, 224			; load olcnt register
OLoop:	ldi		ilcnt, 237			; load ilcnt register
ILoop:	dec		ilcnt				; decrement ilcnt
		brne	ILoop				; Continue Inner Loop
		dec		olcnt				; decrement olcnt
		brne	OLoop				; Continue Outer Loop
		dec		waitcnt				; Decrement wait 
		brne	Loop				; Continue Wait loop	

		pop		olcnt				; Restore olcnt register
		pop		ilcnt				; Restore ilcnt register
		pop		waitcnt				; Restore wait register
		ret							; Return from subroutine

;***********************************************************
;*	Stored Program Data
;***********************************************************

;-----------------------------------------------------------
; An example of storing a string. Note the labels before and
; after the .DB directive; these can help to access the data
;-----------------------------------------------------------
			;	  16-bits	 ;
WELCOME:
	.DB		"Welcome!        "

PRESS_START:
	.DB		"Please press PD7"

WAIT1_START:
	.DB		"Ready. Waiting  "

WAIT2_START:
	.DB		"for the opponent"

GAME_START:
    .DB		"Game Start      "		; Declaring data in ProgMem

GESTURE_ROCK:
	.DB		"Rock            "

GESTURE_PAPER:
	.DB		"Paper           "

GESTURE_SCISSOR:
	.DB		"Scissor         "

VICTORY_START:
	.DB		"You won!        "

LOSS_START:
	.DB		"You lost        "




;***********************************************************
;*	Additional Program Includes
;***********************************************************
.include "LCDDriver.asm"		; Include the LCD Driver

