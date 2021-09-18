/*
 * File:   main.c
 * Author: seebs
 *
 * Created on January 6, 2021, 5:05 PM
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 20000000L
#include <util/delay.h>
#include "lcd_definitions.h"
#include "lcd.h"

/* this trivial implementation uses 0x10-0x1F as control characters.
 * 0x00-0x0F aren't used because the LCD allows them to be programmed
 * as graphics data.
 *
 */

enum LCD_CMDS {
  LCD_NOP = 0x10,
  LCD_GOTO,      // 8 bits: 3-bit line, 5-bit column
  LCD_BACKLIGHT, // 8 bits: brightness
  LCD_SETCGRAM,  // 9 chars: address and CG RAM data
  LCD_CLEAR,     // no bits
};

/* populate CGRAM data with special symbols, using only bottom three bits */
void lcd_cgram(uint8_t addr, const uint8_t data[8])
{
    lcd_command((1 << LCD_CGRAM) | ((addr & 0x07) << 3));
    for (int i = 0; i < 8; ++i) {
        lcd_data(data[i]);
    }
}

static uint8_t cgram_buf[9], cgram_index;

void red(int on) {
    if (on) {
       TCA0.SPLIT.HCMP1 = 0xFF;
    } else {
       TCA0.SPLIT.HCMP1 = 0x0;
    }
}

void green(int on) {
    if (on) {
        TCA0.SPLIT.HCMP2 = 0xFF;
    } else {
        TCA0.SPLIT.HCMP2 = 0x0;        
    }
}

#define TWIBUFSIZ 16
volatile uint8_t twi_head, twi_tail;
volatile uint8_t twi_buf[TWIBUFSIZ];

uint8_t twi_get() {
    if (twi_head == twi_tail) {
        return 0;
    }
    uint8_t c = twi_buf[twi_tail];
    twi_tail = (twi_tail + 1) % TWIBUFSIZ;
    red(0);
    return c;
}

uint8_t twi_ready() {
    uint8_t ready = !(twi_head == twi_tail);
    green(ready);
    return ready;
}

uint8_t twi_put(uint8_t c) {
    if (((twi_head + 1) % TWIBUFSIZ) == twi_tail) {
        red(1);
        return 1;
    }
    twi_buf[twi_head] = c;
    twi_head = (twi_head + 1) % TWIBUFSIZ;
    red(0);
    green(1);
    return 0;
}

ISR(TWI0_TWIS_vect)
{
    if ((TWI0.SSTATUS & (TWI_COLL_bm | TWI_BUSERR_bm)) != 0) {
        return;
    }

    if ((TWI0.SSTATUS & TWI_APIF_bm) && (TWI0.SSTATUS & TWI_AP_bm)) {
        if (TWI0.SSTATUS & TWI_DIR_bm) {
            // Master wishes to read from slave, which we don't support.
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
        } else {
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
        }
        return;
    }
    if (TWI0.SSTATUS & TWI_DIF_bm) {
        if (TWI0.SSTATUS & TWI_DIR_bm) {
            // Master wishes to read from slave
            if (!(TWI0.SSTATUS & TWI_RXACK_bm)) {
                // Received ACK from master
                TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
            } else {
                // Received NACK from master
                TWI0.SSTATUS |= (TWI_DIF_bm | TWI_APIF_bm);
                TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
            }
        } else // Master wishes to write to slave
        {
            // have been sent data. let's take it if we can.
            if (((twi_head + 1) % TWIBUFSIZ) == twi_tail) {
                // ... we can't.
                TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
                return;
            }
            if (twi_put(TWI0.SDATA)) {
                TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
                return;
            }
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
        }
        return;
    }

    // Check if STOP was received
    if ((TWI0.SSTATUS & TWI_APIF_bm) && (!(TWI0.SSTATUS & TWI_AP_bm))) {
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        return;
    }
}

int main(void) {  
    // 20 MHz, no divisor
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0x0);
    PORTA.DIRSET = 0x38;
    PORTB.DIRSET = 0x3C;
    // invert PA3 (because the backlight is controlled by that kind of transistor)
    // _PROTECTED_WRITE(PORTA.PIN3CTRL, PORTA.PIN3CTRL | PORT_INVEN_bm);

    // set Timer A to split mode (because WO2/4/5 are the convenient pins))
    _PROTECTED_WRITE(TCA0.SINGLE.CTRLD, TCA_SPLIT_SPLITM_bm);
    // Set both periods to 255
    _PROTECTED_WRITE(TCA0.SPLIT.HPER, 255);
    _PROTECTED_WRITE(TCA0.SPLIT.LPER, 255);
    // WO3/4/5 are HCMP0/1/2, W0/1/2 are LCMP. We want W03 for backlight, and
    // W04/W05 for the LEDs.
    _PROTECTED_WRITE(TCA0.SPLIT.CTRLB, TCA_SPLIT_HCMP2EN_bm | TCA_SPLIT_HCMP1EN_bm | TCA_SPLIT_HCMP0EN_bm); 
    TCA0.SPLIT.HCMP0 = 0xff;
    TCA0.SPLIT.HCMP1 = 0x0;
    TCA0.SPLIT.HCMP2 = 0x0;
    _PROTECTED_WRITE(TCA0.SPLIT.CTRLA, TCA_SPLIT_ENABLE_bm); // enable

    red(1);
    lcd_init(LCD_DISP_ON);
    green(1);
    lcd_clrscr();
    red(0);

    // TWI setup (mostly stolen from MCC))
    //SDASETUP 4CYC; SDAHOLD OFF; FMPEN disabled; 
    TWI0.CTRLA = 0x00;
    
    //Debug Run
    TWI0.DBGCTRL = 0x00;
    
    //Slave Address
    TWI0.SADDR = 0x64;
    
    //ADDRMASK 0; ADDREN disabled; 
    TWI0.SADDRMASK = 0x00;
    
    //ACKACT ACK; SCMD NOACT; 
    TWI0.SCTRLB = 0x00;
    
    //Slave Data
    TWI0.SDATA = 0x00;
    
    //DIF disabled; APIF disabled; COLL disabled; BUSERR disabled; 
    TWI0.SSTATUS = 0x00;
    
    //DIEN enabled; APIEN enabled; PIEN disabled; PMEN disabled; SMEN disabled; ENABLE enabled; 
    TWI0.SCTRLA = 0xC1;
    
    sei();
    green(0);
    
    uint8_t c;
    uint8_t cmd, prevcmd;
    prevcmd = 0;
    while (1) {
        if (twi_ready()) {
            if (prevcmd != 0) {
                cmd = prevcmd;
                prevcmd = 0;
            } else {
                cmd = twi_get();
            }
            // controls done with control characters, but 0-10 are the character graphics range.
            if (cmd >= 0x20 || cmd < 0x10) {
                lcd_putc(cmd);
                continue;
            }
            switch (cmd) {
            case LCD_NOP: default:
                break;
            case LCD_GOTO:
                // silently fail if there's not a byte available yet
                if (!twi_ready()) {
                    prevcmd = LCD_GOTO;
                    continue;
                }
                c = twi_get();
                lcd_gotoxy(c & 0x1F, (c & ~0x1F) >> 5);
                break;
            case LCD_SETCGRAM:
                if (cgram_index < 9) {
                    if (twi_ready()) {
                        cgram_buf[cgram_index] = twi_get();
                        cgram_index++;
                    }
                }
                if (cgram_index == 9) {
                    lcd_cgram(cgram_buf[0], cgram_buf + 1);
                    cgram_index = 0;
                } else {
                    prevcmd = LCD_SETCGRAM;
                    continue;
                }
                break;
            case LCD_BACKLIGHT:
                if (twi_ready()) {
                    TCA0.SPLIT.HCMP0 = twi_get();
                } else {
                    prevcmd = LCD_BACKLIGHT;
                    continue;
                }
                break;
            case LCD_CLEAR:
                lcd_clrscr();
                break;
            }
        }
    }
}
