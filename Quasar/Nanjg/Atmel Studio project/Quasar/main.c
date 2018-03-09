/*
 * Quasar v2.0 firmware for Nanjg 105C/D flashlight drivers
 * Copyright SinuX 2018
 * License: CC-BY-NC-SA (non-commercial use only, derivative works must be under the same license)
 * Based on DrJones LuxDrv 0.30b
 *
 * Key features:
 * > Up to 16 mode groups, each group may have up to 16 modes
 * > Acts like factory Nanjg 105D 2-group firmware (switch to first mode, wait 2s for blink and click to change modes group)
 * > Uses on-time memory with wear leveling to support lighted tail switches
 * > Additional blinking modes: Police Strobe and SOS
 * > Three memory modes: last, first and next
 * > Low voltage indication
 * > Battcheck: perform 16 fast clicks to display battery percentage (up to 4 blinks for 100%, 75%, 50% and < 25%)
 *
 * Flash command:
 * > avrdude -p t13 -c usbasp -u -Uflash:w:quasar.hex:a -Ulfuse:w:0x75:m -Uhfuse:w:0xFD:m
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
#include <avr/interrupt.h>

/* Setup */
#define byte uint8_t
#define WDTIME 0b01000000
#define sleepinit() do { WDTCR = WDTIME; sei(); MCUCR = (MCUCR & ~0b00111000) | 0b00100000; } while (0) // WDT-int and Idle-Sleep
#define SLEEP asm volatile ("SLEEP")
#define pwminit() do { TCCR0A = 0b00100001; TCCR0B = 0b00000001; } while (0)			// Chan A, phasePWM, clk/1 -> 2.35kHz @ 1.2MHz
#define adcinit() do { ADMUX = 0b01100000 | adcchn; ADCSRA = 0b11000100; } while (0)	// Ref 1.1V, left-adjust, ADC1/PB2; enable, start, clk/16
#define adcread() do { ADCSRA |= 64; while (ADCSRA & 64); } while (0)
#define adcresult ADCH
#define ADCoff ADCSRA &= ~(1 << 7) // ADC off (enable = 0);
#define ADCon  ADCSRA |= (1 << 7)  // ADC on
#define ACoff  ACSR |= (1 << 7)    // AC off (disable = 1)

/* Special modes. Comment out to disable */
#define STROBE		254
#define PSTROBE		253
#define SOS			252
#define BATTCHECK	16			// Amount of fast clicks to trigger BATTCHECK mode

#define MEM_LAST				// Memory mode: MEM_FIRST, MEM_NEXT, MEM_LAST

/* Max groups count - 16, max modes count - 16
 * Use PowerOfTwo values (2, 4, 8, 16) to reduce firmware size */
#define MODES_COUNT			8	// 7 modes per group (last slot is empty)
#define GROUPS_COUNT		2	// 2 groups
#define GROUP_CHANGE_MODE	0	// Mode number for group change blink

PROGMEM const byte groups[GROUPS_COUNT][MODES_COUNT] = {{ 6, 32, 128, 255, 0, 0, 0, 0 },	// Zero slots will be ignored
														{ 6, 32, 128, 255, STROBE, PSTROBE, SOS, 0 }};

														
/* ============================================================================================================================================ */


#ifdef BATTCHECK
	byte shortClicks = 0;
#endif
volatile byte group = 0;
volatile byte mode = 0;
byte ticks = 0;
byte eepos = 0;


/* Get next mode number */
byte getNextMode(void) {
	byte nextMode = (mode + 1) % MODES_COUNT;
	if (!pgm_read_byte(&groups[group][nextMode])) nextMode = 0;
	return nextMode;
}


/* Write word to EEPROM with wear leveling */
void eepSave(byte c, byte g, byte m) {
	cli();	// Disable interrupts
	byte oldpos = eepos;
	eepos = (eepos + 2) & 31; // Wear leveling, use next cell
	
	// Write first byte
	EEARL = eepos; EEDR = c; EECR = 32 + 4; EECR = 32 + 4 + 2; // 32:write only (no erase) 4:enable  2:go
	while (EECR & 2); // Wait for completion
	EEARL = oldpos;	EECR = 16 + 4; EECR = 16 + 4 + 2; // 16:erase only (no write) 4:enable  2:go
	while (EECR & 2); // Wait for completion
	
	// Write second byte
	EEARL = eepos + 1; EEDR = g << 4 | m; EECR = 32 + 4; EECR = 32 + 4 + 2; // 32:write only (no erase) 4:enable  2:go
	while (EECR & 2); // Wait for completion
	EEARL = oldpos + 1; EECR = 16 + 4; EECR = 16 + 4 + 2; // 16:erase only (no write) 4:enable  2:go
	while (EECR & 2); // Wait for completion
	sei();	// Enable interrupts
}


/* Decode group from data (0bGGGG****) */
inline byte decodeGroup(byte data) {
	return (data >> 4) % GROUPS_COUNT;
}


/* Decode mode from data (0b****MMMM) */
inline byte decodeMode(byte data) {
	return (data & 0xf) % MODES_COUNT;
}


/* Read byte from EEPROM */
byte eepReadByte(byte addr) {
	while (EECR & 2);
	EEARL = addr;
	EECR = 1;
	return EEDR;
}


/* Load data from EEPROM and write next mode */
inline void eepLoad(void) {		
	cli();	// Disable interrupts
	byte clicksData;
	while (((clicksData = eepReadByte(eepos)) == 0xff) && (eepos < 30)) eepos++;	// Find first data byte
	byte groupMode = eepReadByte(eepos + 1);	// Read second data byte
	sei();	// Enable interrupts
	
	if (clicksData != 0xff) {
		#ifdef BATTCHECK
			shortClicks = clicksData & 0x7f;
		#endif
		group = decodeGroup(groupMode);
		mode = decodeMode(groupMode);
		
		// Last on-time was short
		if (clicksData & 0x80) {
			mode = getNextMode();
			#ifdef BATTCHECK
				shortClicks++;
			#endif
		}
	}
	
	#ifdef BATTCHECK
		eepSave(shortClicks | 0x80, group, mode); // Write mode, with short-on marker
	#else
		eepSave(0x80, group, mode);
	#endif
}


/* Get and return battery voltage */
byte getBatteryVoltage(void) {
	adcread();
	return adcresult;
}


/* WatchDogTimer interrupt */
ISR(WDT_vect) {
	if (ticks < 255) {
		ticks++;
		
		// Lock mode according to memory type
		if (ticks == LOCKTIME) {
			#ifdef MEM_NEXT
				eepSave(0, group, getNextMode());
			#else
				#ifdef MEM_FIRST
					eepSave(0, group, 0);
				#else
					#ifdef MEM_LAST
						eepSave(0, group, mode);
					#endif
				#endif
			#endif
		}
	}
}


/* Sleep (20 * count) milliseconds */
void doSleep(byte count) {
	while (count--) SLEEP;
}


/* Perform impulses */
void doImpulses(byte count, byte onTime, byte offTime) {
	while (count--) {
		PWM = 255;
		byte time = onTime;
		while (time--) SLEEP;
		PWM = 0;
		time = offTime;
		while (time--) SLEEP;
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
			
	eepLoad();	// Get current group and mode from EEPROM
	byte pmode = pgm_read_byte(&groups[group][mode]);	// Get actual PWM value (or special mode code)
	byte lowbattCounter = 0;
	
	// Display battery level after BATTCHECK fast clicks
	#ifdef BATTCHECK
		if (shortClicks >= BATTCHECK) {
			PWM = 0;
			doSleep(50);
			byte voltage = getBatteryVoltage();
			byte blinksCount;
			if (voltage >= 170) blinksCount = 4;		// < 100%
			else if (voltage >= 160) blinksCount = 3;	// < 75%
			else if (voltage >= 145) blinksCount = 2;	// < 50%
			else blinksCount = 1;						// < 25%
			doImpulses(blinksCount, 10, 20);
			shortClicks = 0;
			doSleep(50);
		}
	#endif

	// Blink for group change
	if (mode == GROUP_CHANGE_MODE) {
		PWM = pmode;
		doSleep(LOCKTIME * 2);
		byte nextGroup = (group + 1) % GROUPS_COUNT;
		eepSave(0, nextGroup, 0);
		PWM = 0;
		doSleep(LOCKTIME / 10);
		PWM = pmode;
		doSleep(LOCKTIME);
		#ifdef MEM_NEXT
			eepSave(0, group, getNextMode());
		#else
			eepSave(0, group, GROUP_CHANGE_MODE);
		#endif
	}

	// Do the work according to current mode
	// ATTENTION: blinking modes don't have low voltage indication
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
			while (1) {
				// Check battery
				#ifdef BATTMON
					if (getBatteryVoltage() < BATTMON) {
						if (++lowbattCounter > 8) {
							pmode = (pmode >> 1) + 3;
							lowbattCounter = 0;
						}
					} else lowbattCounter = 0;
				#endif

				PWM = pmode;
				doSleep(10); // 200ms delay
			}
			
	} // Switch
	
	return 0;
	
} // Main