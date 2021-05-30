/*
 * File:   main.c
 * Author: seebs
 *
 * Created on January 6, 2021, 5:05 PM
 */


#include <avr/io.h>
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

static uint8_t prevcmd;
uint8_t cgram_buf[9], cgram_index;

uint8_t cmds[] = "hello\nhi there";
uint8_t *next = cmds;

int available() {
    return cmds + sizeof(cmds) - next - 1;
}

int receive() {
    if (*next != '\0') {
       return *next++;
    }
    return 0;
}

void red(int on) {
    if (on) {
        _PROTECTED_WRITE(TCA0.SPLIT.HCMP1, 255);
        // PORTA.OUTSET = 0x8;
    } else {
        _PROTECTED_WRITE(TCA0.SPLIT.HCMP1, 0);
        // PORTA.OUTSET = 0x8;
    }
}

void green(int on) {
    if (on) {
        _PROTECTED_WRITE(TCA0.SPLIT.HCMP2, 255);
        // PORTA.OUTSET = 0x20;
    } else {
        _PROTECTED_WRITE(TCA0.SPLIT.HCMP2, 0);
        // PORTA.OUTCLR = 0x20;
    }
}

int main(void) {  
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0x0);
    _PROTECTED_WRITE(TCA0.SINGLE.CTRLD, TCA_SPLIT_SPLITM_bm);
    _PROTECTED_WRITE(TCA0.SPLIT.HPER, 255);
    _PROTECTED_WRITE(PORTA.PIN3CTRL, PORTA.PIN3CTRL | PORT_INVEN_bm);
    // WO3/4/5 are HCMP0/1/2
    _PROTECTED_WRITE(TCA0.SINGLE.CTRLB, TCA_SPLIT_HCMP2EN_bm | TCA_SPLIT_HCMP1EN_bm | TCA_SPLIT_HCMP0EN_bm);
    _PROTECTED_WRITE(TCA0.SPLIT.HCMP0, 0);
    _PROTECTED_WRITE(TCA0.SPLIT.HCMP1, 0);
    _PROTECTED_WRITE(TCA0.SPLIT.HCMP2, 0);
    _PROTECTED_WRITE(TCA0.SPLIT.CTRLA, TCA_SPLIT_ENABLE_bm); // enable
    PORTA.DIRSET = 0x38;
    /* Replace with your application code */
    unsigned char h = TCA0.SPLIT.HCMP2;
    red(1);
    lcd_init(LCD_DISP_ON_CURSOR_BLINK);
    green(1);
    lcd_clrscr();
    red(0);
    lcd_gotoxy(3, 2);
    green(0);
    
    while (available() > 0) {
        uint8_t c;
        if (prevcmd != 0) {
            c = prevcmd;
            prevcmd = 0;
        } else {
            c = receive();
        }
        // controls done with control characters, but 0-10 are the character graphics range.
        if (c >= 0x20 || c < 0x10) {
            lcd_putc(c);
            continue;
        }
        switch (c) {
        case LCD_NOP: default:
            break;
        case LCD_GOTO:
            // silently fail if there's not a byte available yet
            if (available() < 1) {
                prevcmd = c;
                break;
            }
            c = *next++;
            lcd_gotoxy(c & 0x1F, (c & ~0x1F) >> 5);
            break;
        case LCD_SETCGRAM:
            while (cgram_index < 9) {
            if (available() < 1) {
                prevcmd = LCD_SETCGRAM;
                break;
            }
            cgram_buf[cgram_index] = receive();
          }
          lcd_cgram(cgram_buf[0], cgram_buf + 1);
          cgram_index = 0;
          break;
        case LCD_BACKLIGHT:
            if (available() == 1) {
                c = receive();
            }
            _PROTECTED_WRITE(TCA0.SPLIT.HCMP0, c);
            break;
        case LCD_CLEAR:
            lcd_clrscr();
            break;
        }
    }
    uint8_t toggle = 0;
    while (1) {
        _delay_ms(50L);
        toggle = (toggle + 1) % 16;
        red(toggle&0x4);
        green(toggle&0x8);
        h++;
        _PROTECTED_WRITE(TCA0.SPLIT.HCMP0, h);
    }
}
