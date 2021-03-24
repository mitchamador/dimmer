#include <avr/wdt.h>

static __inline__
__attribute__ ((__always_inline__))
void _wdt_enable (const uint8_t value, const uint8_t prescale)
{
	__asm__ __volatile__(
		"cli" "\n\t"
		"wdr" "\n\t"
		"ldi r16, %1" "\n\t"
		"ldi r17, %2" "\n\t"
		"out %0, r16" "\n\t"
		"out %0, r17" "\n \t"
		: /* no outputs */
		: "I"(_SFR_IO_ADDR(_WD_CONTROL_REG)),
		  "M"((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
		  "M"((uint8_t)(value | ((prescale & 0x08 ? _WD_PS3_MASK : 0x00) | (prescale & 0x07))))
		: "r0");
}
