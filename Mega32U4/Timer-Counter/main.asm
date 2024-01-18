.include "m32u4def.inc"

.def mpr = r16 ; Define a temporary register
.def mpr2 = r17

.org $0000 ; Reset vector
    rjmp INIT

.org $0026 ; Timer1 Compare A vector
    jmp TIMER1_COMPARE_A_ISR

INIT:
    ; Initialize Stack Pointer
    ldi mpr, low(RAMEND)
    out SPL, mpr
    ldi mpr, high(RAMEND)
    out SPH, mpr

    ; Initialize PORTB for output
    ldi mpr, 0xFF ; Set all bits of PORTB as output
    out DDRB, mpr

    ; Initialize Timer/Counter1 in CTC mode with a 256 prescaler
    ldi mpr, (1 << WGM12) | (1 << CS12) ; CTC mode, 256 prescaler
    sts TCCR1B, mpr

    ; Set the compare value for a 1-second delay
    ldi mpr, high($3D08)
    sts OCR1AH, mpr
    ldi mpr, low($3D08)
    sts OCR1AL, mpr

    ; Enable Timer/Counter1 compare match A interrupt
    ldi mpr, (1 << OCIE1A)
    sts TIMSK1, mpr

    ; Enable global interrupts
    sei

    ldi mpr, 0b10000000
    out PORTB, mpr

MAIN_LOOP:
    rjmp MAIN_LOOP ; Main loop (do nothing)

TIMER1_COMPARE_A_ISR:
    ; Toggle PB7 and PB4
    in mpr, PORTB
    ldi mpr2, 0b10010000
    eor mpr, mpr2
    out PORTB, mpr

    reti ; Return from interrupt
