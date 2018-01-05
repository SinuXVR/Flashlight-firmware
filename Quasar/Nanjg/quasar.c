/*
 * Quasar v1.0 firmware for Nanjg 105C/D flashlight drivers
 * Copyright SinuX 2018
 * License: CC-BY-NC-SA (non-commercial use only, derivative works must be under the same license)
 * Based on DrJones LuxDrv 0.30b
 *
 * Key features:
 * > Up to 16 mode groups, each group may include up to 8 modes
 * > Acts like factory Nanjg 105D 2-group firmware (switch to first mode, wait 2s for blink and click to change modes group)
 * > Uses on-time memory with wear leveling to support lighted tail switches
 * > Additional blinking modes: Police Strobe and SOS
 * > Three memory modes: last, first and next
 * > Low voltage indication
 * > Battcheck: perform 10-16 fast clicks to display battery percentage (up to 4 blinks for 100%, 75%, 50% and < 25%)
 *
 * Flash command:
 * > avrdude -p t13 -c usbasp -u -Uflash:w:quasar.hex:a -Ulfuse:w:0x75:m -Uhfuse:w:0xFF:m
 */

#define F_CPU 4800000	// CPU: 4.8MHz  PWM: 9.4kHz

#define LOCKTIME 50		// Time in 1/50s until a mode gets locked, e.g. 50/50 = 1s
#define BATTMON  125	// Enable battery monitoring with this threshold

/* IO pins */
#define outpin 1		// PWM out pin
#define PWM OCR0B		// PWM-value
#define adcpin 2		// Battery monitoring pin
#define adcchn 1
#define portinit() do { DDRB = (1 << outpin); PORTB = 0xff - (1 << outpin) - (1 << adcpin); } while (0)

/* Dependencies */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

/* Setup */
#define byte uint8_t
#define word uint16_t
#define WDTIME 0b01000000
#define sleepinit() do { WDTCR = WDTIME; sei(); MCUCR = (MCUCR & ~0b00111000) | 0b00100000; } while (0) // WDT-int and Idle-Sleep
#define SLEEP asm volatile ("SLEEP")
#define pwminit() do { TCCR0A = 0b00100001; TCCR0B = 0b00000001; } while (0)  // Chan A, phasePWM, clk/1 -> 2.35kHz @ 1.2MHz
#define adcinit() do { ADMUX = 0b01100000 | adcchn; ADCSRA = 0b11000100; } while (0) // Ref 1.1V, left-adjust, ADC1/PB2; enable, start, clk/16
#define adcread() do { ADCSRA |= 64; while (ADCSRA & 64); } while (0)
#define adcresult ADCH
#define ADCoff ADCSRA &= ~(1 << 7) // ADC off (enable = 0);
#define ADCon  ADCSRA |= (1 << 7)  // ADC on
#define ACoff  ACSR |= (1 << 7)    // AC off (disable = 1)

/* Special modes. Comment out to disable */
#define STROBE		254
#define PSTROBE		253
#define SOS			252
#define BATTCHECK

/* Groups and modes */
#define MEM_LAST			// Memory mode: MEM_FIRST, MEM_NEXT, MEM_LAST
#define MODES_COUNT		7	// 7 modes per group
#define GROUPS_COUNT	2	// 2 groups
// Max groups count - 16, max modes count - 8
PROGMEM const byte groups[GROUPS_COUNT][MODES_COUNT] = {{ 6, 32, 128, 255, 0, 0, 0 },	// Zero slots will be ignored
														{ 6, 32, 128, 255, STROBE, PSTROBE, SOS }};

														
/* ============================================================================================================================================ */


volatile byte mypwm = 0;
volatile byte group = 0;
volatile byte mode = 0;

byte ticks = 0;
byte pmode = 0;
byte eep[32];	// EEPROM buffer
byte eepos = 0;
byte lowbattCounter = 0;

#ifdef BATTCHECK
	word battcheckCounter __attribute__ ((section (".noinit")));
#endif


/* Extract group num from byte (*GGGG***b) */
inline byte decodeGroup(byte data) {
	return (data & 0x78) >> 3;
}


/* Extract memory num from byte (*****MMMb) */
inline byte decodeMode(byte data) {
	return data & 0x7;
}


/* Combine group and mode to single byte (0GGGGMMMb), max group num - 15, max mode num - 7 */
inline byte codeGroupAndMode(byte a, byte b) {
	return a << 3 | b;
}


/* Write byte to EEPROM with wear leveling */
void eepSave(byte data) {
	byte oldpos = eepos;
	eepos = (eepos + 1) & 31; // Wear leveling, use next cell
	EEARL = eepos; EEDR = data; EECR = 32 + 4; EECR = 32 + 4 + 2; // 32:write only (no erase) 4:enable  2:go
	while (EECR & 2); // Wait for completion
	EEARL = oldpos; EECR = 16 + 4; EECR = 16 + 4 + 2; // 16:erase only (no write) 4:enable  2:go
}


/* Get next mode number */
byte getNextMode(void) {
	byte nextMode = mode + 1;
	if (nextMode >= MODES_COUNT || !pgm_read_byte(&groups[group][nextMode])) nextMode = 0;
	return nextMode;
}


/* Get and return battery voltage */
byte getBatteryVoltage(void) {
	adcread();
	return adcresult;
}


/* WatchDogTimer interrupt */
ISR(WDT_vect) {
	if (ticks < 255) ticks++;
	
	// Lock mode according to memory type
	if (ticks == LOCKTIME) {
		#ifdef MEM_NEXT
			eepSave(codeGroupAndMode(group, getNextMode()));
		#else
			#ifdef MEM_FIRST
				eepSave(codeGroupAndMode(group, 0));
			#else
				#ifdef MEM_LAST
					eepSave(codeGroupAndMode(group, mode));
				#endif
			#endif
		#endif
		
		#ifdef BATTCHECK
			battcheckCounter = 0;
		#endif
	}

	// Check battery every 100ms
	#ifdef BATTMON
		if (getBatteryVoltage() < BATTMON) {
			if (++lowbattCounter > 8) {
				mypwm = (mypwm >> 1) + 3;
				lowbattCounter = 0;
			}
		} else lowbattCounter = 0;
	#endif
}


/* Read current group and mode from EEPROM and write next mode */
inline void getmode(void) {
	eeprom_read_block(&eep, 0, 32);							// Read block
	while ((eep[eepos] == 0xff) && (eepos < 32)) eepos++;	// Find data byte
	
	byte groupModeData = eep[eepos];
	group = decodeGroup(groupModeData);
	if (group >= GROUPS_COUNT) group = 0;
	mode = decodeMode(groupModeData);
	if (mode >= MODES_COUNT) mode = 0;

	// Last on-time was short
	if (groupModeData & 0x80) {
		groupModeData &= 0x7f;
		mode = getNextMode();
		#ifdef BATTCHECK
			if (battcheckCounter != 0xaaaa)
				battcheckCounter = (~battcheckCounter & 1) | (battcheckCounter << 1);
		#endif
	}
	
	eepSave(codeGroupAndMode(group, mode) | 0x80); // Write mode, with short-on marker
}


/* Sleep (20 * count) milliseconds */
void doSleep(byte count) {
	while (count--) SLEEP;
}


/* Perform impulses
 * ATTENTION : this method doesn't have low voltage step down */
inline void doImpulses(byte count, byte onTime, byte offTime) {
	while (count--) {
		PWM = 255;
		doSleep(onTime);
		PWM = 0;
		doSleep(offTime);
	}
}


/* The main program */
int main(void) {
	portinit();
	sleepinit();
	ACoff;
	
	#if defined(BATTMON) || defined(BATTCHECK)
		adcinit();
	#else
		ADCoff;
	#endif
	
	pwminit();
	
	getmode();	// Get current group and mode from EEPROM
	pmode = pgm_read_byte(&groups[group][mode]);	// Get actual PWM value (or special mode code)
	mypwm = pmode;
	
	// Display battery level after 10-16 fast clicks
	#ifdef BATTCHECK
		if (battcheckCounter == 0xaaaa) {
			PWM = 0;
			doSleep(50);
			byte voltage = getBatteryVoltage();
			byte blinksCount;
			if (voltage >= 170) blinksCount = 4;		// < 100%
			else if (voltage >= 160) blinksCount = 3;	// < 75%
			else if (voltage >= 145) blinksCount = 2;	// < 50%
			else blinksCount = 1;						// < 25%
			doImpulses(blinksCount, 10, 20);
			doSleep(50);
			battcheckCounter = 0;
		}
	#endif
	
	// Blink at first mode for group change
	if (mode == 0) {
		PWM = mypwm;
		doSleep(100);
		byte nextGroup = group + 1;
		nextGroup = nextGroup % GROUPS_COUNT;
		eepSave(codeGroupAndMode(nextGroup, 0));
		PWM = 0;
		doSleep(5);
		PWM = mypwm;
		doSleep(50);
		#ifdef MEM_NEXT
			eepSave(codeGroupAndMode(group, getNextMode()));
		#else
			eepSave(codeGroupAndMode(group, 0));
		#endif
	}

	// Do the work according to current mode
	switch (pmode) {
		// Strobe
		#ifdef STROBE
			case STROBE:
				while (1) {
					doImpulses(1, 1, 2);
				} break;
		#endif

		// Police Strobe
		#ifdef PSTROBE
			case PSTROBE:
				while (1) {
					doImpulses(5, 1, 2);
					doSleep(50);
				} break;
		#endif

		// Simple SOS
		#ifdef SOS
			case SOS:
				while (1) {
					doImpulses(3, 5, 12);
					doSleep(25);
					doImpulses(3, 25, 25);
					doSleep(12);
					doImpulses(3, 5, 12);
					doSleep(100);
				} break;
		#endif

		// All other: use as PWM value
		default:
			mypwm = pmode;
			while (1) {
				PWM = mypwm;
				doSleep(10);
			}
			
	} // Switch
	
	return 0;
	
} // Main