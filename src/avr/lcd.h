#ifndef lcd_h
#define lcd_h

/*
 * All characters in the rom are ASCII-encoded, 8 bits
 *
 * LCD interfacing functions in 4-bit mode
 * Make sure to init lcd before using it by calling lcd_init(),
 * Write lcd_send_string("") to write to lcd,
 * lcd_clear to clear
 * Cursor does not automatically reset btw!
 * Set it by lcd_return_cursor_home or by clearing.
 * To write to row 2 only change ddram addr with lcd_set_ddram(char c);
 */

#define ROW1ADR 0x00
#define ROW2ADR 0x40

void lcd_init(void);
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_return_cursor_home(void);
void lcd_set_ddram(char c);

#endif
