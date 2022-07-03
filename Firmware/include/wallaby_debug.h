#ifndef WALLABY_DEBUG_H_
#define WALLABY_DEBUG_H_

void wallaby_debug_init(void);
void wallaby_debug_register_command(const char *command, void (*function)(const char *const));
void wallaby_debug_printf(const char *format, ...);
void wallaby_debug_poll(void);

#endif