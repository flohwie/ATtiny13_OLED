
; ******************************************************************************
; *  Assembler tests with ATtiny 13                                            *
; *  09.01.2021-28.01.2021                                                     *
; *  (C)2021 by Flohwie; based on many tutorials:                              *
; *                                                                            *
; *  Credits go  to:                                                           *
; *  Excellent assembler tutorials:                                            *
; *    https://www.avr-asm-tutorial.net - Gerhard Schmidt (DG4FAC)             *
; *    https://www.mikrocontroller.net                                         *
; *    https://www.rahner-edu.de - Richard Rahner                              *
; *    http://www.rjhcoding.com/avr-asm-delay-subroutine.php                   *
; *    http://darcy.rsgc.on.ca/ACES/TEI4M/AVRdelay.html                        *
; *  Perfect ressources for learning I2C:                                      *
; *    https://i2c.info/i2c-bus-specification                                  *
; *    http://davbucci.chez-alice.fr - Davide Bucci (pic_i2c.inc)              *
; *  Very good reading / code for OLED:                                        *
; *    https://jayconsystems.com/blog/getting-started-with-oled-displays       *
; *    https://github.com/Sylaina/oled-display                                 *
; *    http://stefanfrings.de/arduino_oled/index.html                          *
; *    https://iotexpert.com/debugging-ssd1306-display-problems                *
; *  Datasheets:                                                               *
; *    https://ww1.microchip.com/downloads/en/devicedoc/doc2535.pdf            *
; *    https://www.crystalfontz.com/controllers/SolomonSystech/SSD1306/339/    *
; *  for checking delays and debugging the SCL/SDA lines:                      *
; *    https://github.com/EspoTek/Labrador                                     *
; *    https://jyetech.com/dso-150-shell-oscilloscope                          *
; ******************************************************************************


.nolist
;.include "tn13def.inc"  ; Header file with definitions for ATTINY13
.device ATtiny13A        ; for compiling with gavrasm gerd's AVR assembler
                         ; include file is not needed!
.list


; =========================================================
;   D E S C R I P T I O N 
; =========================================================

; when the push-button is pressed the OLED display
; will show the text "ATtiny13   :-)" for 2.5 seconds
; then the text will start scrolling, after 2 seconds 
; the watchdog will be set and
; after 8 seconds, the watchdog will reset the ATtiny 
; and the display will be cleared.

; this is an educational project,
; I2C is realized with bitbanging over 2 pins,
; the most basic OLED functions are implemented 
; and there is a rudimentary "font" table.
; it uses 954 bytes (i.e. 93.2% flash memory),
; so there is only marginal room left for something
; more meaningful.
; but this project helped in learning and understanding
; basic principles of programming an AVR in assembler and:
; it was fun!


; =========================================================
;   H A R D W A R E  I N F O R M A T I O N
; =========================================================

; ATMEL ATtiny13
;                                +--\/--+
;  (PCINT5/RESET/ADC0/dW)  PB5  1|      |8  VCC  (1.8-5.5 V)
;  (PCINT3/CLKI/ADC3)      PB3  2|      |7  PB2  (SCK/ADC1/T0/PCINT2)
;  (PCINT4/ADC2)           PB4  3|      |6  PB1  (MISO/AIN1/OC0B/INT0/PCINT1)
;  (0 V)                   GND  4|      |5  PB0  (MOSI/AIN0/OC0A/PCINT0)
;                                +------+

; The Microchip ATtiny13A is a 
; picoPower 8-bit AVR RISC-based microcontroller featuring:
; 1 KB of ISP Flash
; 64-byte EEPROM
; 64-byte SRAM
; 32-Byte register file,
; and 4-channel 10-bit A/D converter
; operation at 1.8-5.5 V

; OLED 0.9" I2C White Color
; is a cheap Chinese module
; made of 128x64 individual write OLED pixels,
; it uses I2C to communicate with microcontroller
; and is drivern by SSD1306 controller
; Power: 3.3 to 5.0 V
; Dimensions: 0.96 x 0.75 in


; =========================================================
;   C L O C K  S E T T I N G S  (F U S E S)
; =========================================================

; with default device settings as shipped: 
; CKSEL = "10", SUT = "10", and CKDIV8 programmed.
; clock source: Internal RC Oscillator running at 9.6 MHz
; initial system clock prescaling of 8

; CPU clock and the I/O clock: 1.2 MHz


; =========================================================
;   C O N N E C T I O N S 
; =========================================================

;  PB4 ----> Push button ----> GND
;  PB0 ----> SCL-pin of 0.9"-OLED module (I2C)
;  PB0 ----> 470 Ohm ----> GND (blue LED1 for debugging)
;  PB1 ----> SDA-pin of 0.9"-OLED module (I2C)
;  PB1 ----> 470 Ohm ----> GND (blue LED2 for debugging)

;  CAVE: during programming with avrdude, the OLED display
;        has to be disconnected under some circumstances


; =========================================================
;   D E F I N I T I O N S  &  C O N S T A N T S
; =========================================================

;.equ XTAL = 1200000                              ; system clock frequency (for delays)

; formula for 500 us delay loop: iCnt = (0.0005 * fclk - 4) / 3
; or: ( XTAL * 500 ) / 3 ) / 1000000, which is not exact but portable with XTAL parameter
.equ iCnt = 199                                  ; loop value for 500 us delay routine

; formula for 10 ms delay loop: iVal = (0.01 * fclk - 8) / 4
.equ iVal = 2998                                 ; inner loop value for 10 ms delay routine


.equ i2c_PORT = PORTB
.equ i2c_DDR = DDRB
.equ SCL = PORTB0
.equ SDA = PORTB1

.equ SDIN_PORT = PINB
.equ SDIN = PINB1

.equ I2C_ADDRESS  = 0x78                         ; OLED I2C write address
                                                 ; (7-bit: 0x3C & R/W-bit = 0 --> 0x78 = 0b01111000)
.equ CTRL_COMMAND = 0x00                         ; SSD1306 control byte for command stream
.equ CTRL_DATA    = 0x40                         ; SSD1306 control byte for data stream
.equ OLED_BYTES   = 0x400                        ; 128 x 64 = 8192 / 8 - 1= 1024 = 1024

.def rACK = R20                                  ; register holding ACK/NACK bit
.def rTMP = R22                                  ; temp register holding byte for I2C protocol
.def rMP1 = R16                                  ; multi-purpose register 1
.def rMP2 = R17                                  ; multi-purpose register 2
.def rCNT = R18                                  ; loop counter

.def loopCt  = R19                               ; delay loop count for 10 ms delay
.def iLoopRl = R24                               ; inner loop register low
.def iLoopRh = R25                               ; inner loop register high


; =========================================================
;   I 2 C   M A C R O S
; =========================================================

.macro I2C_begin_transmission
    rcall i2c_start                              ; initiate communication on I2C bus
    ldi rTMP, @0                                 ; I2C write address
    rcall i2c_send_byte                          ; send a byte on I2C bus and receive ACK bit
    ldi rTMP, @1                                 ; send command or data stream
    rcall i2c_send_byte
.endmacro

.macro I2C_end_transmission
    rcall i2c_stop                               ; stop communication on I2C bus
; switch off SDA and SCL ports if not needed
    in rMP1, i2c_PORT                            ; load PORT register settings
    andi rMP1, ~((1<<SCL) | (1<<SDA))            ; switch off SDA and SCL lines (set to low)
    out i2c_PORT, rMP1
.endmacro

.macro I2C_write_byte
    ldi rTMP, @0
    rcall i2c_send_byte                          ; send a byte on I2C bus
.endmacro

; send 6 bytes from program memory
; all chars consist of 6 bytes,
; the first byte is always 0x00;
; the macro variables define the table and the offset
; the loops saves a whopping 126 bytes here!
.macro transmit_char
  ldi ZL, low(@0*2+@1)                           ; LSB-pointer + offset, @0: table in font_table .db, @1: offset
  ldi ZH, high(@0*2+@1)                          ; init Z pointer to storage bytes (MSB-pointer)
  ldi  rMP2, 6                                   ; 6 bytes for each character
  char_loop:
    dec rMP2
    lpm rTMP, Z+                                 ; load from program memory to register and simultaneously increment pointer 
    rcall i2c_send_byte
    brne char_loop
.endmacro

; send 6 0x00 bytes for each space
.macro transmit_space
  ldi rTMP, 0x00
  ldi  rMP2, @0*6                                 ; 6 bytes for each space
  byte_loop:
    dec rMP2
    rcall i2c_send_byte
    brne byte_loop
.endmacro


; =========================================================
;   O L E D   M A C R O S
; =========================================================

; set cursor to specific page, and define start and column
.macro OLED_set_cursor
  ldi rTMP, @0                                   ; set page 0xB0 + y
  rcall i2c_send_byte
  ldi rTMP, @1                                   ; column address (0x21)
  rcall i2c_send_byte
  ldi rTMP, @2                                   ; first column, x (e.g. 0x00, 0x01 or 0x10 ...)
  rcall i2c_send_byte
  ldi rTMP, @3                                   ; last column = width - 1 (0x7F for full line)
  rcall i2c_send_byte
.endmacro

; start horizontal (right or left) scrolling 
.macro OLED_start_scrolling
  ldi rTMP, @0                                   ; set right (0x26) or left (0x27) scrolling
  rcall i2c_send_byte
  ldi rTMP, 0x00                                 ; send dummy byte 0x00
  rcall i2c_send_byte
  ldi rTMP, @1                                   ; start page for scrolling (0x00-0x07)
  rcall i2c_send_byte
  ldi rTMP, @2                                   ; set time interval between each scroll step in frames
                                                 ; (0h=5, 1h=64, 2h=128, 3h=256, 4h=3, 5h=4, 6h=5, 7h=2 frames)
  rcall i2c_send_byte
  ldi rTMP, @3                                   ; end page for scrolling (0x01-0x07), must be larger than start page
  rcall i2c_send_byte
  ldi rTMP, 0x00                                 ; send dummy byte 0x00
  rcall i2c_send_byte
  ldi rTMP, 0xFF                                 ; send dummy byte 0xFF
  rcall i2c_send_byte
  ldi rTMP, 0x2F                                 ; start scrolling
  rcall i2c_send_byte
.endmacro


; =========================================================
;   D E L A Y   M A C R O S
; =========================================================

; set cursor to specific page, and define start and column
.macro delayms
  push loopCt
  push iLoopRl
  push iLoopRh

  ldi loopCt, @0/10
  rcall delay10ms

  pop iLoopRh
  pop iLoopRl
  pop loopCt
.endmacro



; =========================================================
;   M A I N  P R O G R A M
; =========================================================

    .CSEG
    .ORG 0x00

; initiate stack
    ldi rMP1, LOW(RAMEND)                        ; set stack pointer to SRAM end (RAMEND = 0x00df)
    out SPL, rMP1                                ; set stack pointer (I/O register; SPL = 0x3d)

; setup ports
    sbi i2c_DDR, SCL                             ; make SCL output
    sbi i2c_DDR, SDA                             ; make SDA output

    sbi PORTB, PB4                               ; activate pull-up for PB4 (remains input)

; enable sleep
    ldi rMP1, (1<<SE)                            ; sleep enabled (SE= 5; Sleep Enable)
    out MCUCR, rMP1                              ; MCUCR - MCU Control Register
    sei                                          ; set interrupt flag

; turn off watchog (if a system reset occured it is still active!)
    cli                                          ; disable interrupts
    wdr                                          ; reset Watchdog Timer
    in rMP1, MCUSR                               ; read MCUSR - MCU Status register
    andi rMP1, ~(1<<WDRF)                        ; delete Watchdog Reset Flag, keep other bits unchanged
    out MCUSR, rMP1
    in rMP1, WDTCR                               ; read Watchdog Timer Control Register
    ori rMP1, (1<<WDCE) | (1<<WDE)               ; Set WDCE and WDE, keep other bits unchanged
    out WDTCR, rMP1
    andi rMP1, ~(1<<WDE)                         ; stop watchdog
    out WDTCR, rMP1
    sei                                          ; enable interrupts

; Setup flow for OLED
    I2C_begin_transmission I2C_ADDRESS, CTRL_COMMAND

    I2C_write_byte 0xA8                          ; set Multiplex Ratio
    I2C_write_byte 0x3F                          ; = 64-1 (display heigth in pixels)

    I2C_write_byte 0xA1                          ; set segment re-map

    I2C_write_byte 0xC8                          ; set COM Output Scan Direction


    I2C_write_byte 0x8D                          ; enable charge pump, set DC-DC enable
    I2C_write_byte 0x14                          ; use internal vcc

    I2C_write_byte 0x20                          ; set addressing mode
    I2C_write_byte 0x00                          ; horizontal addressing mode

    I2C_write_byte 0x2E                          ; stop scrolling

    I2C_write_byte 0xAF                          ; display on

    I2C_end_transmission

; Clear display
    I2C_begin_transmission I2C_ADDRESS, CTRL_COMMAND
    OLED_set_cursor 0xB0, 0x21, 0x00, 0x7F
    I2C_end_transmission

    I2C_begin_transmission I2C_ADDRESS, CTRL_DATA
    ; send 1024 0x00 bytes :-)
    ldi rTMP, 0x00
    ldi YH, high(OLED_BYTES)                     ; load MSB register with the upper byte (.def YH = r29), 1024
    ldi YL, low(OLED_BYTES)                      ; load LSB register with the upper byte (.def YL = r28)
    clear:
      rcall i2c_send_byte
      sbiw YL, 1                                 ; decrease double register value by one
      brne clear

    I2C_end_transmission

    ldi rCNT, 0x00                               ; set loop counter to zero after reset


loop:
    sbic PINB, PB4                               ; Skip next instruction if bit in I/O register is cleared (pull-up input pin is high, because button is open)
                                                 ; only when button is pressed, input pin will be low, and the next instruction
                                                 ; will be skipped, i.e. the OLED display will be turned on
    rjmp button_off                              ; button is open, input is high so input-pin bit is set, so jump

    cpi rCNT, 0x00                               ; compare the counter value with zero, skio next command if rCNT is NOT zero
    brne loop                                    ; after the first loop, rCNT will be 0x01, so jump back ("very primitive debouncing")
                                                 ; this actually ends up in an infinite loop that will be reset by the watchdog


; set cursor to page 3 and column to 0x10:
    I2C_begin_transmission I2C_ADDRESS, CTRL_COMMAND
;                   page  col   start end
    OLED_set_cursor 0xB3, 0x21, 0x10, 0x7F
    I2C_end_transmission

; Send letters "ATtiny13" to display
    I2C_begin_transmission I2C_ADDRESS, CTRL_DATA
    transmit_char capital_letters, 0             ; A
    transmit_char capital_letters, 114           ; T
    transmit_char lowercase_letters, 12          ; t
    transmit_char lowercase_letters, 0           ; i
    transmit_char lowercase_letters, 6           ; n
    transmit_char lowercase_letters, 18          ; y
    transmit_char digits, 6                      ; 1
    transmit_char digits, 18                     ; 3
    transmit_space 3                             ; "   "
    transmit_char symbols, 0                     ; :
    transmit_char symbols, 6                     ; -
    transmit_char symbols, 12                    ; )
    I2C_end_transmission

; wait for 2.5 seconds
    delayms 2500                                 ; delay for 250 x 10ms

; start scrolling
    I2C_begin_transmission I2C_ADDRESS, CTRL_COMMAND
;                   right/left  start speed end
    OLED_start_scrolling 0x27, 0x00, 0x07, 0x07
    I2C_end_transmission

; wait for 2 seconds
    delayms 2000                                 ; delay for 200 x 10ms

; watchdog_on:
    cli
    ldi rMP1, (1<<WDP3) | (1<<WDCE) | (1<<WDE) | (1<<WDP0)
    out WDTCR, rMP1                              ; WDTCR - Watchdog Timer Control Register
                                                 ; turn on watchdog and set 4s time-out
                                                 ; WDCE: Watchdog Change Enable, WDE: Watchdog System Reset Enable
                                                 ; WDP3, and WDP0: Watchdog Timer Prescaler 3 and 0 --> 8 seconds
    sei

; increase loop counter (simple form of "debouncing")
    inc rCNT

    rjmp loop



; =========================================================
;   I 2 C  B U S  R O U T I N E S   ( B I T B A N G I N G )
; =========================================================

; when the I2C bus is not used, both lines are kept high
; the microcontroller STARTS the communication
; by putting a low state on SDA (high-to-low) while SCL is kept high
i2c_start:

; initiate the start condition on the I2C bus
    sbi i2c_PORT, SDA                            ; SDA goes high
    sbi i2c_PORT, SCL                            ; SCL goes high
    rcall short_delay

; SDA high to low transition when SCL is high
    cbi i2c_PORT, SDA                            ; SDA goes low, SCL remains high
    rcall short_delay

; SCL goes low; then leave SDA and SCL low
    cbi i2c_PORT, SCL                            ; SCL goes low
    rcall short_delay
    ret


; the microcontroller STOPS the communication
; by putting a low-to-high transition on SDA while SCK is kept high
i2c_stop:

; initiate the stop condition on the I2C bus
    cbi i2c_PORT, SDA                            ; SDA goes low
    cbi i2c_PORT, SCL                            ; SCL goes low
    rcall short_delay

; SCL goes high; SDA remains low
    sbi i2c_PORT, SCL                            ; SCL goes high
    rcall short_delay

; SDA low to high transition when SCL is high
    sbi i2c_PORT, SDA                            ; SDA goes high, SCL remains high
    rcall short_delay

    ret


; during data transmission, the SDA line must be set up only when the SCL line is low
; bits are sent one by one, starting with the most significant one (MSB)
; the minimum clock cycle time time should be 2.5 us (SSD1306 data sheet), which
; means a 400 kHz frequency. The delays programmed here ensure this, 
; one SCl pulse is 13 us, the total cycle is ~30 us, meaning a frequency of ~ 33 kHz.
; This could be optimized of course but I wanted to learn and see the SCL/SDA lines
; on my DSO/Labrador toys.
; after each completed byte, the receiver sends an acknowledgment (ACK):
; the master relinquishes the SDA line and the ACK bit is sent by the receiver
; by pulling SDA at a low state.
; If the SDA remains high, this means NOT acknowledged (NACK).
; Handling of NACK is not implemented yet in the code
; (the master should issue a stop condition on the I2C bus!).
i2c_send_byte:

; transmit 8 bits, MSB first
.macro clock_out_bit
   cbi i2c_PORT, SCL                             ; SCL goes low, SDA changes allowed
   sbrs rTMP, @0                                 ; skip next command if bit n is set in rTMP
   cbi i2c_PORT, SDA                             ; SDA goes low (bit is not set)
   sbrc rTMP, @0                                 ; skip next command if bit n is cleared in rTMP
   sbi i2c_PORT, SDA                             ; SDA goes high (bit is set)
   rcall short_delay                             ; ensure that SDA line is stable before SCL goes high
   sbi i2c_PORT, SCL                             ; SCL goes high
   rcall short_delay                             ; ensure that data clocks out for a minimum of 2.5 us
.endmacro

    clock_out_bit(7)                             ; MSB first
    clock_out_bit(6)
    clock_out_bit(5)
    clock_out_bit(4)
    clock_out_bit(3)
    clock_out_bit(2)
    clock_out_bit(1)
    clock_out_bit(0)                             ; LSB last

; read ACK bit from slave
    cbi i2c_PORT, SCL                            ; SCL goes low, SDA changes allowed
    sbi i2c_PORT, SDA                            ; SDA goes high
    cbi i2c_DDR, SDA                             ; make SDA pin INPUT
    sbi i2c_PORT, SCL                            ; SCL goes high
    rcall short_delay                            ; ensure that ACK bit clocks in (minimum of 2.5 us)

    ldi rACK, 0x00                               ; 0x00 means "Acknowledgment", no error
    sbic SDIN_PORT, SDIN                         ; skip next instruction if SDA is low (SDA was pulled low by slave, which means Acknowledgment!, rACK remains 0x00)
    ldi rACK, 0xFF                               ; 0xFF (no Acknowledgment, NACK), because SDA remains high

    cbi i2c_PORT, SCL                            ; SCL goes low
    sbi i2c_DDR, SDA                             ; make SDA pin OUTPUT again
    rcall short_delay

    ret


; =========================================================
;   D E L A Y  R O U T I N E S
; =========================================================

; modified from 
; Assembly code auto-generated
; by utility from Bret Mulvey
; ~500 us at 1.2 MHz
; 1 + iCnt * ( 1 + 2) -1 + 4 = 3 * iCnt + 4 clocks
; for 0.0005 s = 0.5 ms = 500 us: iCnt = (0.0005 * fclk - 4) / 3
; @ 1.2 MHz: 500.8 us (iCnt = 199)
long_delay:
    ldi  rMP2, iCnt                              ; 1 clock
    long_delay_:
      dec rMP2                                   ; 1 clock
      brne long_delay_                           ; 2 clocks or 1 (final)
      ret                                        ; final: 4 clocks

; for exactly 5 us on ATtiny13:
; (LPM is a good 3-clock cycles, 1-word delay!)
; the SSD1306 needs at least 2.5 us
; total duration including rcall overhead (3 clocks):
; 3 + 2 * 3 + 4 = 13 clocks
; add 2 clocks for cbi command, until SCL goes low
; pulse duration should be 15 clocks - imho
; @ 1.2 MHz: 1 clock = 0.833 us; 15 clocks = 12.5 us
; SCK-pulse measured with Labrador and DS0150: 13 us, woww not bad! ;-)
short_delay:
    lpm                                          ; 3 clocks
    lpm                                          ; 3 clocks
    ret                                          ; 4 clocks

; for exactly 10 ms on ATm328p:
; from: http://www.rjhcoding.com/avr-asm-delay-subroutine.php 
; inner loop duration: D_in = iVal * (2 + 2) - 1 = iVal * 4 - 1
; outer loop duration: D_ou = loopCt * (1 + 1 + D_in + 1 + 2) - 1 = loopCt * (4 * iVal - 1 + 5) - 1
; total duration: D_ou + 1 + 4 = loopCt * (4 * iVal + 4) + 4
; loopCt = 1: will need 4 * iVal + 8 cycles
; @16 Mhz: 0.01 s = 10 ms (iVal = 39998)
; @1.2 MhZ: 0.01 s = 10 ms (iVal = 2998)
; formula for iVal: iVal = (0.01 * fclk - 8) / 4
delay10ms:
    ldi iLoopRl, LOW(iVal)                       ; intialize inner loop count in inner                                       1 clock
    ldi iLoopRh, HIGH(iVal)                      ; loop high and low registers                                               1 clock
    iLoop:
      sbiw iLoopRl, 1                            ; decrement inner loop registers                      2 clocks
      brne iLoop                                 ; branch to iLoop if iLoop registers != 0             2 clocks or 1 (final)
      dec loopCt                                 ; decrement outer loop register                                             1 clock
      brne delay10ms                             ; branch to outer loop if outer loop register != 0                          2 clocks or 1 (final)
      nop                                        ; no operation                                                                                         final: 1 clock
      ret                                        ; return from subroutine                                                                               final: 4 clocks


; =========================================================
;   O T H E R  R O U T I N E S
; =========================================================

button_off:
    rjmp loop


; =========================================================
;   "F O N T"  T A B L E
; =========================================================

capital_letters:
.db  0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C          ; A (0)
.db  0x00, 0x7F, 0x49, 0x49, 0x49, 0x36          ; B (6)
.db  0x00, 0x3E, 0x41, 0x41, 0x41, 0x22          ; C (12)
.db  0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C          ; D (18)
.db  0x00, 0x7F, 0x49, 0x49, 0x49, 0x41          ; E (24)
.db  0x00, 0x7F, 0x09, 0x09, 0x09, 0x01          ; F (30)
.db  0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A          ; G (36)
.db  0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F          ; H (42)
.db  0x00, 0x00, 0x41, 0x7F, 0x41, 0x00          ; I (48)
.db  0x00, 0x20, 0x40, 0x41, 0x3F, 0x01          ; J (54)
.db  0x00, 0x7F, 0x08, 0x14, 0x22, 0x41          ; K (60)
.db  0x00, 0x7F, 0x40, 0x40, 0x40, 0x40          ; L (66)
.db  0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F          ; M (72)
.db  0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F          ; N (78)
.db  0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E          ; O (84)
.db  0x00, 0x7F, 0x09, 0x09, 0x09, 0x06          ; P (90)
.db  0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E          ; Q (96)
.db  0x00, 0x7F, 0x09, 0x19, 0x29, 0x46          ; R (102)
.db  0x00, 0x46, 0x49, 0x49, 0x49, 0x31          ; S (108)
.db  0x00, 0x01, 0x01, 0x7F, 0x01, 0x01          ; T (114)
.db  0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F          ; U (120)
.db  0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F          ; V (126)
.db  0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F          ; W (132)
.db  0x00, 0x63, 0x14, 0x08, 0x14, 0x63          ; X (138)
.db  0x00, 0x07, 0x08, 0x70, 0x08, 0x07          ; Y (144)
.db  0x00, 0x61, 0x51, 0x49, 0x45, 0x43          ; Z (150)

lowercase_letters:
; .db  0x00, 0x20, 0x54, 0x54, 0x54, 0x78                ; a
; .db  0x00, 0x7F, 0x48, 0x44, 0x44, 0x38                ; b
; .db  0x00, 0x38, 0x44, 0x44, 0x44, 0x20                ; c
; .db  0x00, 0x38, 0x44, 0x44, 0x48, 0x7F                ; d
; .db  0x00, 0x38, 0x54, 0x54, 0x54, 0x18                ; e
; .db  0x00, 0x08, 0x7E, 0x09, 0x01, 0x02                ; f
; .db  0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C                ; g
; .db  0x00, 0x7F, 0x08, 0x04, 0x04, 0x78                ; h
.db  0x00, 0x00, 0x44, 0x7D, 0x40, 0x00                ; i
; .db  0x00, 0x40, 0x80, 0x84, 0x7D, 0x00                ; j
; .db  0x00, 0x7F, 0x10, 0x28, 0x44, 0x00                ; k
; .db  0x00, 0x00, 0x41, 0x7F, 0x40, 0x00                ; l
; .db  0x00, 0x7C, 0x04, 0x18, 0x04, 0x78                ; m
.db  0x00, 0x7C, 0x08, 0x04, 0x04, 0x78                ; n
; .db  0x00, 0x38, 0x44, 0x44, 0x44, 0x38                ; o
; .db  0x00, 0xFC, 0x24, 0x24, 0x24, 0x18                ; p
; .db  0x00, 0x18, 0x24, 0x24, 0x18, 0xFC                ; q
; .db  0x00, 0x7C, 0x08, 0x04, 0x04, 0x08                ; r
; .db  0x00, 0x48, 0x54, 0x54, 0x54, 0x20                ; s
.db  0x00, 0x04, 0x3F, 0x44, 0x40, 0x20                ; t
; .db  0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C                ; u
; .db  0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C                ; v
; .db  0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C                ; w
; .db  0x00, 0x44, 0x28, 0x10, 0x28, 0x44                ; x
.db  0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C                ; y
; .db  0x00, 0x44, 0x64, 0x54, 0x4C, 0x44                ; z


digits:
.db  0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E          ; 0
.db  0x00, 0x00, 0x42, 0x7F, 0x40, 0x00          ; 1
.db  0x00, 0x42, 0x61, 0x51, 0x49, 0x46          ; 2
.db  0x00, 0x21, 0x41, 0x45, 0x4B, 0x31          ; 3
.db  0x00, 0x18, 0x14, 0x12, 0x7F, 0x10          ; 4
.db  0x00, 0x27, 0x45, 0x45, 0x45, 0x39          ; 5
.db  0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30          ; 6
.db  0x00, 0x01, 0x71, 0x09, 0x05, 0x03          ; 7
.db  0x00, 0x36, 0x49, 0x49, 0x49, 0x36          ; 8
.db  0x00, 0x06, 0x49, 0x49, 0x29, 0x1E          ; 9

symbols:
;.db  0x00, 0x36, 0x46, 0x50, 0x46, 0x36          ; :-)
.db  0x00, 0x00, 0x36, 0x36, 0x00, 0x00          ; :
.db  0x00, 0x08, 0x08, 0x08, 0x08, 0x08          ; -
.db  0x00, 0x00, 0x41, 0x22, 0x1c, 0x00          ; )

; =========================================================
;                   end of source code
; =========================================================


    ; Bitbanging an I2C OLED module with an ATtiny13.
    ; Copyright (C) 2021 by Wolfgang Schmidt (aka Flohwie)

    ; This program is free software: you can redistribute it and/or modify
    ; it under the terms of the GNU General Public License as published by
    ; the Free Software Foundation, either version 3 of the License, or
    ; (at your option) any later version.

    ; This program is distributed in the hope that it will be useful,
    ; but WITHOUT ANY WARRANTY; without even the implied warranty of
    ; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    ; GNU General Public License for more details.

    ; You should have received a copy of the GNU General Public License
    ; along with this program.  If not, see <https://www.gnu.org/licenses/>.