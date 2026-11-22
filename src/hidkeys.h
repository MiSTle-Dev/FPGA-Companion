#ifndef HIDKEYS_H
#define HIDKEYS_H

// convert strings to hid keycodes and back
uint8_t hidkeys_get_code(const char *str);
uint8_t hidkeys_get_code_with_modifier(const char *str, uint8_t *modifier);
const char *hidkeys_get_name(uint8_t code);
int hidkeys_get_modifier_name(char *str, uint8_t mod);

// convert strings to gamepad button maps and back
int hidkeys_get_btnmap(const char *str);
int hidkeys_get_btnmap_str(char *str, uint16_t map);

#endif // HIDKEYS_H
