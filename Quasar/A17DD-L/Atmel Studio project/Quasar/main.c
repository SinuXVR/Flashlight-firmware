/*
 * Quasar v2.0 firmware for A17DD-L FET+1 flashlight drivers
 * Copyright SinuX 2018
 * License: CC-BY-NC-SA (non-commercial use only, derivative works must be under the same license)
 * Based on DrJones LuxDrv 0.30b
 *
 * Key features:
 * > Up to 16 mode groups, each group may have up to 16 modes
 * > Acts like factory Nanjg 105D 2-group firmware (switch to first mode, wait 2s for blink and click to change modes group)
 * > Off-time memory with wear leveling
 * > Additional blinking modes: Police Strobe and SOS
 * > Three memory modes: last, first and next
 * > Low voltage indication
 * > Battcheck: perform 16 fast clicks to display battery percentage (up to 4 blinks for 100%, 75%, 50% and < 25%)
 *
 * Flash command:
 * > avrdude -p t13 -c usbasp -u -Uflash:w:quasar.hex:a -Ulfuse:w:0x75:m -Uhfuse:w:0xFD:m
 */

#define F_CPU 4800000	// CPU: 4.8MHz  PWM: 9.4kHz

/* IO pins */
#define amcpin 0			// PWM out pin for AMC - PB0
#define fetpin 1			// PWM out pin for FET - PB1
#define AMC_PWM OCR0A		// PWM-value on AMC pin
#define FET_PWM OCR0B		// PWM-value on FET pin
#define batpin 2			// Battery monitoring pin - PB2
#define cappin 3			// OTC pin - PB3
#define batchn 1			// Battery ADC channel - ADC1
#define capchn 3			// OTC ADC channel - ADC3

/* Dependencies */
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

/* Setup */
#define byte uint8_t
#define sbyte int8_t
#define WDTIME 0b01000000
#define portinit() do { DDRB = (1 << fetpin) | (1 << amcpin); PORTB = 0xff - (1 << amcpin) - (1 << fetpin) - (1 << batpin) - (1 << cappin); } while (0)
#define sleepinit() do { WDTCR = WDTIME; sei(); MCUCR = (MCUCR & ~0b00111000) | 0b00100000; } while (0) // WDT-int and Idle-Sleep
#define SLEEP asm volatile ("SLEEP")
#define pwminit() do { TCCR0A = 0b10100001; TCCR0B = 0b00000001; FET_PWM = 0; AMC_PWM = 0; } while (0)  // Chan A & B, phasePWM, clk/1 -> 2.35kHz @ 1.2MHz
#define batadcinit() do { ADMUX = 0b01100000 | batchn; ADCSRA = 0b11000100; } while (0)	// Ref 1.1V, left-adjust, ADC1/PB2; enable, start, clk/16
#define capadcinit() do { ADMUX = 0b01100000 | capchn; ADCSRA = 0b11000100; } while (0)	// Ref 1.1V, left-adjust, ADC3/PB3; enable, start, clk/16
#define chargecap() do { DDRB |= (1 << cappin); PORTB |= (1 << cappin); } while (0)
#define adcread() do { ADCSRA |= 64; while (ADCSRA & 64); } while (0)
#define adcresult ADCH
#define ADCoff ADCSRA &= ~(1 << 7) // ADC off (enable = 0)
#define ACoff  ACSR |= (1 << 7)    // AC off (disable = 1)

/* Special modes. Comment out to disable */
#define STROBE			126
#define PSTROBE			125
#define SOS				124
#define BATTMON			125	// Enable battery monitoring with this threshold
#define BATTCHECK		16	// Amount of fast clicks to trigger BATTCHECK mode, comment out to disable
#define TURBO_TIMEOUT	60	// Comment out to disable, 60 ~ 60s

/* Memory settings */
#define MEM_LAST			// Memory mode: MEM_FIRST, MEM_NEXT, MEM_LAST
#define CAP_THRESHOLD	190	// Threshold voltage on the OTC
#define LOCKTIME 50			// Time in 1/50s until a group gets locked after blink, e.g. 50/50 = 1s

/* Max groups count - 16, max modes count - 16
 * Use PowerOfTwo values (2, 4, 8, 16) to reduce firmware size */
#define MODES_COUNT			8	// 7 modes per group (last slot is empty)
#define GROUPS_COUNT		2	// 2 groups
#define GROUP_CHANGE_MODE	0	// Mode number for group change blink

/* Groups and modes definition
 * Negative values - for AMC, positive - for FET
 * Values range: -127...+127, e.g. -127 value means 255 on AMC pin, +127 means 255 on FET
 * Zero slots at the end of group will be ignored */
PROGMEM const sbyte groups[GROUPS_COUNT][MODES_COUNT] = {{ -3, -127, 64, 127, 0, 0, 0, 0 },
														 { -3, -127, 64, 127, STROBE, PSTROBE, SOS, 0 }};

														
/* ============================================================================================================================================ */


#ifdef BATTCHECK
	byte shortClicks = 0;
#endif
byte group = 0;
byte mode = 0;
byte eepos = 0;
#ifdef TURBO_TIMEOUT
	byte turboTicks = 0;
#endif


/* WatchDogTimer interrupt */
ISR(WDT_vect) {
	// Do nothing
}


/* Sleep (20 * count) milliseconds */
void doSleep(byte count) {
	while (count--) SLEEP;
}


/* Get and return ADC value */
byte getADCResult(void) {
	adcread();
	return adcresult;
}


/* Get next mode number */
byte getNextMode(void) {
	byte nextMode = (mode + 1) % MODES_COUNT;
	if (!pgm_read_byte(&groups[group][nextMode])) nextMode = 0;
	return nextMode;
}


/* Write word to EEPROM with wear leveling */
void eepSave(byte c, byte g, byte m) {
	byte oldpos = eepos;
	eepos = (eepos + 2) & 31; // Wear leveling, use next cell
	
	// Write first byte
	while (EECR & 2); // Wait for completion
	EEARL = eepos; EEDR = c; EECR = 32 + 4; EECR = 32 + 4 + 2; // 32:write only (no erase) 4:enable  2:go
	while (EECR & 2); // Wait for completion
	EEARL = oldpos;	EECR = 16 + 4; EECR = 16 + 4 + 2; // 16:erase only (no write) 4:enable  2:go
	
	// Write second byte
	while (EECR & 2); // Wait for completion
	EEARL = eepos + 1; EEDR = g << 4 | m; EECR = 32 + 4; EECR = 32 + 4 + 2; // 32:write only (no erase) 4:enable  2:go
	while (EECR & 2); // Wait for completion
	EEARL = oldpos + 1; EECR = 16 + 4; EECR = 16 + 4 + 2; // 16:erase only (no write) 4:enable  2:go
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
	byte clicksData;
	while (((clicksData = eepReadByte(eepos)) == 0xff) && (eepos < 30)) eepos++;	// Find first data byte
	byte groupMode = eepReadByte(eepos + 1);	// Read second data byte
	
	if (clicksData != 0xff) {
		#ifdef BATTCHECK
			shortClicks = clicksData;
		#endif
		group = decodeGroup(groupMode);
		mode = decodeMode(groupMode);
		
		// Last on-time was short
		getADCResult();
		if (getADCResult() > CAP_THRESHOLD) {
			mode = getNextMode();
			#ifdef BATTCHECK
				shortClicks++;
			#endif
		} else {
			#ifdef MEM_NEXT
				mode = getNextMode();
			#else
				#ifdef MEM_FIRST
					mode = 0;
				#endif
			#endif
			shortClicks = 0;
		}
	}
	
	#ifdef BATTCHECK
		eepSave(shortClicks, group, mode); // Write mode, with short-on marker
	#else
		eepSave(0, group, mode);
	#endif
	
	// Charge up OTC
	chargecap();
}


/* Perform impulses */
void doImpulses(byte count, byte onTime, byte offTime) {
	while (count--) {
		FET_PWM = 255;
		byte time = onTime;
		while (time--) SLEEP;
		FET_PWM = 0;
		time = offTime;
		while (time--) SLEEP;
	}
}


/* Set PWM value on pins */
void setPWM(sbyte value) {
	FET_PWM = 0;
	AMC_PWM = 0;
	
	if (value > 0) {
		FET_PWM = (value << 1) + 1;
	} else if (value < 0) {
		AMC_PWM = (-value << 1) + 1;
	}
}


/* The main program */
int main(void) {
	portinit();
	sleepinit();
	ACoff;
	
	capadcinit();
			
	eepLoad();	// Get current group and mode from EEPROM
	byte pmode = pgm_read_byte(&groups[group][mode]);	// Get actual PWM value (or special mode code)
	
	#ifdef BATTCHECK
		byte lowbattCounter = 0;
	#endif
		
	#if defined(BATTMON) || defined(BATTCHECK)
		batadcinit();
	#else
		ADCoff;
	#endif
		
	pwminit();
	
	// Display battery level after BATTCHECK fast clicks
	#ifdef BATTCHECK
		if (shortClicks >= BATTCHECK) {
			FET_PWM = 0;
			doSleep(50);
			byte voltage = getADCResult();
			byte blinksCount;
			if (voltage >= 170) blinksCount = 4;		// < 100%
			else if (voltage >= 160) blinksCount = 3;	// < 75%
			else if (voltage >= 145) blinksCount = 2;	// < 50%
			else blinksCount = 1;						// < 25%
			doImpulses(blinksCount, 10, 20);
			doSleep(50);
			eepSave(0, group, mode);
		}
	#endif

	// Blink for group change
	if (mode == GROUP_CHANGE_MODE) {
		setPWM(pmode);
		doSleep(LOCKTIME * 2);
		byte nextGroup = (group + 1) % GROUPS_COUNT;
		eepSave(0, nextGroup, GROUP_CHANGE_MODE);
		setPWM(0);
		doSleep(LOCKTIME / 10);
		setPWM(pmode);
		doSleep(LOCKTIME);
		eepSave(0, group, GROUP_CHANGE_MODE);
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
					if (getADCResult() < BATTMON) {
						if (++lowbattCounter > 8) {
							pmode = (pmode >> 1) + 3;
							lowbattCounter = 0;
						}
					} else lowbattCounter = 0;
				#endif
				
				// TURBO timer
				#ifdef TURBO_TIMEOUT
					if (turboTicks < TURBO_TIMEOUT) {
						turboTicks++;
					} else {
						if (pmode == 127) {
							pmode = pmode >> 1;
						}
					}
				#endif
				
				setPWM(pmode);
				doSleep(50); // 1s delay
			}
			
	} // Switch
	
	return 0;
	
} // Main