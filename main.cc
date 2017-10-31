#include <avr/io.h>
#include "lcd.h"
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/eeprom.h>

#define CUTTERPOSSWITCH !(PINC & ( 1 << PC2))

#define CENTERPRESSED !(PINB & ( 1 << PB5))
#define LEFTPRESSED (!(PINB & ( 1 << PB4)))
#define RIGHTPRESSED (PINC & ( 1 << PC3))

#define TOGGLELED (PORTB ^= ( 1 << PB0))

#define CUTTERON PORTC &= ~( 1 << PC1); PORTB |= ( 1 << PB2)
#define CUTTEROFF PORTB |= ( 1 << PB2);PORTC |= ( 1 << PC1)
#define CUTTERREVERSE PORTC |= ( 1 << PC1); PORTB &= ~( 1 << PB2)

#define STEPPER_ON PORTC |= ( 1 << PC5)
#define STEPPER_OFF PORTC &= ~(1 << PC5)

#define MMPERSTEP 0.22
#define SWITCHDEBOUNCE 100

uint32_t motorTiming = 0;


void lcd_puti(int num)
{
	char buf[32];
	lcd_puts(itoa(num,buf,10));
}

typedef enum stepper_directions_m{
	RETRACT,
	EXTRACT
} stepperDirections;

typedef enum lcdMenuNames_m{
	LCD_START,
	LCD_SETUP_LENGTH,
	LCD_SETUP_ISOLATION,
	LCD_SETUP_SPEED,
	LCD_SETUP_ISOLATE_DEPTH,
	LCD_SETUP_NUMBER
} lcdMenuNames;

typedef enum buttonsState_m{
	NOT_PRESSED,
	CENTER_PRESSED,
	LEFT_PRESSED,
	RIGHT_PRESSED
} buttonSate;


struct cableExtruder
{
	uint16_t speed; //mm/s
	uint32_t length; //mm
	uint32_t motorTimeout;
	uint16_t unisolatedLength; // mm
	uint16_t targetLength;
	uint16_t count;
	uint16_t number;
	stepperDirections dir;
};

struct cableExtruder cableEx;
typedef enum motor_states_m{
	STOP,
	CUT,
	CUT_ISO,
	CUTTING,
	CUTTING_ISO,
	REVERSE,
	CALIBRATE
} motorState;

typedef enum cable_making_state{
	WAIT,
	START,
	EXTRUDING_FRONT,
	EXTRUDING_BACK,
	PREPARE_ISOLATE_FRONT,
	PREPARE_ISOLATE_BACK,
	RETRACT_FRONT,
	RETRACT_FRONT_FINISH,
	OPEN_CUTTER,
	EXTRUDING_MAIN,
	CHOP,
	DONE
} cableMakingStates;


// this is for generating the steps
ISR(TIMER0_OVF_vect)
{
	static uint16_t tickCounter = 0;
	tickCounter++;

	if (motorTiming > 0)
	{
		motorTiming--;
	}

	if (tickCounter >= cableEx.speed)
	{
		if (cableEx.length > 0)
		{
			PORTD ^= ( 1 << PD7); // creating a step
			cableEx.length--;
		} else {
			PORTD &= ~( 1 << PD7);
			cableEx.length = 0;
		}

		if (cableEx.dir == EXTRACT)
		{
			PORTD |= ( 1 << PD6);
		} else {
			PORTD &= ~( 1 << PD6);
		}
		tickCounter = 0;

	}
}


char * getMakingStateName(cableMakingStates* makingState)
{
	switch (*makingState)
	{
	case WAIT:
		return "Ready";
		break;
	case START:
		return "Starting..";
		break;
	case EXTRUDING_FRONT:
		return "extrude cable f";
		break;
	case PREPARE_ISOLATE_BACK:
	case PREPARE_ISOLATE_FRONT:
		return "cut isolation";
		break;
	case CHOP:
		return "cutting cable";
		break;
	case EXTRUDING_BACK:
		return "extrude cable b";
		break;
	case EXTRUDING_MAIN:
		return "extrude cable m";
		break;
	default:
		return "unknown";
		break;
	}
}

bool uiHandling(cableMakingStates*  makingState, cableExtruder* cableEx, buttonSate bState)
{
	static lcdMenuNames lcdMenu = LCD_START;
	bool ret = false;

	switch (lcdMenu)
	{
	case LCD_START:
		lcd_clrscr();
		if ( *makingState != WAIT)
		{
			lcd_puts("Cable Cutter\n");
			lcd_puts(getMakingStateName(makingState));
		} else {
			lcd_puts("L: ");
			lcd_puti((int)cableEx->targetLength);
			lcd_puts (" I: ");
			lcd_puti((int)cableEx->unisolatedLength);
			lcd_puts (" C: ");
			lcd_puti((int)cableEx->number);

			lcd_puts ("\n");
			lcd_puts("Start  Setup  ");
			if (bState == LEFT_PRESSED)
			{
				*makingState = START;
			} else if (bState == CENTER_PRESSED)
			{
				lcdMenu = LCD_SETUP_LENGTH;
			} else if (bState == RIGHT_PRESSED)
			{

			}
		}
		break;

	case LCD_SETUP_LENGTH:
		lcd_clrscr();
		lcd_puts("Length (mm): ");
		lcd_puti((int)cableEx->targetLength);
		lcd_puts ("\n");
		lcd_puts("+     Enter    -");
		if (bState == LEFT_PRESSED)
		{
			cableEx->targetLength++;
		} else if (bState == CENTER_PRESSED)
		{
			lcdMenu = LCD_SETUP_ISOLATION;
		} else if (bState == RIGHT_PRESSED)
		{
			cableEx->targetLength--;
		}
		break;

	case LCD_SETUP_ISOLATION:
			lcd_clrscr();
			lcd_puts("Isolate (mm): ");
			lcd_puti((int)cableEx->unisolatedLength);
			lcd_puts ("\n");
			lcd_puts("+     Enter    -");
			if (bState == LEFT_PRESSED)
			{
				cableEx->unisolatedLength++;
			} else if (bState == CENTER_PRESSED)
			{
				lcdMenu = LCD_SETUP_SPEED;
			} else if (bState == RIGHT_PRESSED)
			{
				cableEx->unisolatedLength--;
			}
			break;

	case LCD_SETUP_SPEED:
			lcd_clrscr();
			lcd_puts("Speed : ");
			lcd_puti((int)cableEx->speed);
			lcd_puts ("\n");
			lcd_puts("+     Enter    -");
			if (bState == LEFT_PRESSED)
			{
				cableEx->speed++;
			} else if (bState == CENTER_PRESSED)
			{
				lcdMenu = LCD_SETUP_ISOLATE_DEPTH;
			} else if (bState == RIGHT_PRESSED)
			{
				cableEx->speed--;
			}
			break;

	case LCD_SETUP_ISOLATE_DEPTH:
			lcd_clrscr();
			lcd_puts("Depth : ");
			lcd_puti((int)cableEx->motorTimeout);
			lcd_puts ("\n");
			lcd_puts("+     Enter    -");
			if (bState == LEFT_PRESSED)
			{
				cableEx->motorTimeout += 10;
			} else if (bState == CENTER_PRESSED)
			{
				lcdMenu = LCD_SETUP_NUMBER;
			} else if (bState == RIGHT_PRESSED)
			{
				cableEx->motorTimeout -= 10;
			}
			break;

	case LCD_SETUP_NUMBER:
				lcd_clrscr();
				lcd_puts("# Cables : ");
				lcd_puti((int)cableEx->number);
				lcd_puts ("\n");
				lcd_puts("+     Enter    -");
				if (bState == LEFT_PRESSED)
				{
					cableEx->number += 1;
				} else if (bState == CENTER_PRESSED)
				{
					lcdMenu = LCD_START;
					ret = true;
				} else if (bState == RIGHT_PRESSED)
				{
					cableEx->number -= 1;
				}
				break;


	default:
		lcd_clrscr();
		lcd_puts("unknown menu point..");
		break;
	}

	return ret;
}


struct cableExtruder eeCableEx EEMEM;

buttonSate checkButtons()
{
	static unsigned int pressButtonCenterDebounce = 0;
	static unsigned int pressButtonLeftDebounce = 0;
	static unsigned int pressButtonRightDebounce = 0;

	if (CENTERPRESSED)
	{
		pressButtonCenterDebounce++;
		if (pressButtonCenterDebounce >= SWITCHDEBOUNCE)
		{
			pressButtonCenterDebounce = SWITCHDEBOUNCE;
		}
	} else {
		if (pressButtonCenterDebounce >= SWITCHDEBOUNCE)
		{
			pressButtonCenterDebounce = 0;
			return CENTER_PRESSED;
		}
		pressButtonCenterDebounce = 0;
	}

	if (LEFTPRESSED)
	{
		pressButtonLeftDebounce++;
		if (pressButtonLeftDebounce >= SWITCHDEBOUNCE)
		{
			pressButtonLeftDebounce = SWITCHDEBOUNCE;
		}

	} else {
		if (pressButtonLeftDebounce >= SWITCHDEBOUNCE)
		{
			pressButtonLeftDebounce = 0;
			return LEFT_PRESSED;
		}
		pressButtonLeftDebounce = 0;
	}

	if (RIGHTPRESSED)
	{
		pressButtonRightDebounce++;
		if (pressButtonRightDebounce >= SWITCHDEBOUNCE)
		{
			pressButtonRightDebounce = SWITCHDEBOUNCE;
		}

	} else {
		if (pressButtonRightDebounce >= SWITCHDEBOUNCE)
		{
			pressButtonRightDebounce = 0;
			return RIGHT_PRESSED;
		}
		pressButtonRightDebounce = 0;

	}

	return NOT_PRESSED;
}

int main()
{
	unsigned int mainLoopCounter = 0;
	unsigned int switchDebounce = 0;

	motorState mState = STOP;
	cableMakingStates makingState = WAIT;
	buttonSate bState = NOT_PRESSED;


	cableEx.speed = 45;
	cableEx.length =0;
	cableEx.dir = EXTRACT;
	cableEx.unisolatedLength = 5.0;
	cableEx.targetLength = 40.0;
	cableEx.motorTimeout = 1500;
	cableEx.number = 1;

	// read setting from eeprom
	eeprom_read_block((void*)&cableEx, (void*)&eeCableEx,sizeof(struct cableExtruder));

	if ( (int) cableEx.targetLength == -1)
	{
		//restore defaults
		cableEx.speed = 45;
		cableEx.length =0;
		cableEx.dir = EXTRACT;
		cableEx.unisolatedLength = 5.0;
		cableEx.targetLength = 40.0;
		cableEx.motorTimeout = 1500;
		cableEx.number = 1;
		eeprom_write_block((void*)&cableEx, (void*)&eeCableEx,sizeof(struct cableExtruder));


	}


	//IO
	DDRB |= ( 1 << PB0); //LED
	DDRB |= ( 1 << PB2); //PWM Cutter
	DDRD |= ( 1 << PD6); //DIR
	DDRD |= ( 1 << PD7); //Step

	PORTB |= ( 1 << PB4); // Button up
	PORTB |= ( 1 << PB5); // Button down
	PORTC |= ( 1 << PC3); // Button enter
	PORTC |= ( 1 << PC2); // PU for Cutter Pos switch


	DDRC |= (( 1 << PC1));
	DDRC |= (( 1 << PC5)); // sleep mode


	// pwm init
    //TCCR1A = (1<<WGM10)|(1<<COM1B1);  // Wave Form Generation is Fast PWM 8 Bit,
    //TCCR1B = (1<<WGM12)|(1<<CS11) |( 1 << CS10);     // OC1A and OC1B are cleared on compare match
          //  |(1<<CS10);               // and set at BOTTOM. Clock Prescaler is 1024.

    //OCR1B = 0;
	//timer init stepper
	TCCR0 |= ( 1 << CS01); // Prescaling 8
	TIMSK |= ( 1 << TOIE0); // Interrupt enable
	sei();

	if (CUTTERPOSSWITCH)
	{
		mState = STOP;
	} else {
		mState = CALIBRATE;
	}

	//UI LCD INIT
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();

	while(1)
	{

		bState = checkButtons();
		if (bState != NOT_PRESSED)
		{
			if ( uiHandling(&makingState, &cableEx, bState) == true)
			{
				eeprom_write_block((void*)&cableEx, (void*)&eeCableEx,sizeof(struct cableExtruder));
			}
		}

		if (!( mainLoopCounter % 4000))
		{
			uiHandling(&makingState, &cableEx, bState);
		}
//			lcd_clrscr();
//			//show motor state in first line
//			lcd_puts("CUTTER: ");
//			switch (mState)
//			{
//			case STOP:
//				lcd_puts("STOP");
//				break;
//			case CUT_ISO:
//				lcd_puts("ISO");
//				break;
//			case REVERSE:
//				lcd_puts("REV");
//				break;
//			case CUT:
//				lcd_puts("CUT");
//				break;
//			case CUTTING:
//				lcd_puts("CUT*");
//				break;
//			case CALIBRATE:
//				lcd_puts("CAL");
//				break;
//
//			default:
//				lcd_puts("unknown");
//				break;
//
//			}
//			lcd_puts("\n");
//			switch (makingState) {
//				case WAIT:
//					lcd_puts("WAITING");
//					break;
//				case EXTRUDING_MAIN:
//					lcd_puts("EXT_MAIN");
//					break;
//				case EXTRUDING_BACK:
//					lcd_puts("EXT_BACK");
//					break;
//				case EXTRUDING_FRONT:
//					lcd_puts("EXT_FRONT");
//					break;
//				case CHOP:
//					lcd_puts("CHOP");
//					break;
//				case DONE:
//					lcd_puts("FINISH");
//					break;
//
//				default:
//					break;
//			}
//
//			if (cableEx.length > 0)
//			{
//				lcd_puts(" +");
//
//			}else {
//				lcd_puts(" -");
//			}
//			//PORTB ^= ( 1 << PB0);
//			//PORTD ^= ( 1 << PD6); //DIR
//
//		}

		switch (makingState) {
			case WAIT:
				cableEx.count = cableEx.number;
				STEPPER_OFF;
			break;

			case START:
				STEPPER_ON;
				_delay_ms(100);
				cableEx.length = (uint32_t)(cableEx.unisolatedLength / MMPERSTEP);
				makingState=EXTRUDING_FRONT;
			break;

			case EXTRUDING_FRONT:
				if ( cableEx.length == 0)
				{
					mState = CUT_ISO;
					makingState = PREPARE_ISOLATE_FRONT;
				}
				break;

			case RETRACT_FRONT:
				if (mState == STOP)
				{
					STEPPER_ON;
					cableEx.dir = RETRACT;
					cableEx.length = (uint32_t)(cableEx.unisolatedLength / MMPERSTEP);
					makingState=RETRACT_FRONT_FINISH;
				}
				break;

			case RETRACT_FRONT_FINISH:
				if ( cableEx.length == 0)
				{
					mState = REVERSE;
					cableEx.dir = EXTRACT;
					makingState = PREPARE_ISOLATE_FRONT;
				}
				break;

			case PREPARE_ISOLATE_FRONT:
				if (mState == STOP)
				{
					STEPPER_ON;
					cableEx.length = (uint32_t)((cableEx.targetLength - ( 2 * cableEx.unisolatedLength))/ MMPERSTEP);
					makingState=EXTRUDING_MAIN;
				}
				break;

			case EXTRUDING_MAIN:
				if ( cableEx.length == 0)
				{
					mState = CUT_ISO;
					makingState = PREPARE_ISOLATE_BACK;

				}
				break;



			case PREPARE_ISOLATE_BACK:
				if (mState == STOP)
				{
					cableEx.length = (uint32_t)(cableEx.unisolatedLength / MMPERSTEP);
					makingState=EXTRUDING_BACK;

				}
				break;

			case EXTRUDING_BACK:
				if ( cableEx.length == 0)
				{
					mState = CUT;
					makingState = CHOP;
				}
				break;

			case CHOP:
				if (mState == STOP)
				{
					makingState = DONE;
				}
				break;
			case DONE:
				cableEx.count--;
				if ( cableEx.count == 0)
				{
					makingState = WAIT; // and stop
					STEPPER_OFF;
				} else {
					makingState =START; // next cable
				}
				break;

			default:
				break;
		}

		// motor control Statemachine
		switch (mState)
		{
			case STOP:
				CUTTEROFF;

				break;
			case CUT:
				CUTTERON;

				if (CUTTERPOSSWITCH)
				{
					switchDebounce = 0;
				} else {
					switchDebounce++;
				}

				if (switchDebounce < SWITCHDEBOUNCE)
				{
					mState = CUT;
				} else {

					mState = CUTTING;
				}
				break;
			case CUT_ISO:
				CUTTERON;

				if (CUTTERPOSSWITCH)
				{
					switchDebounce = 0;
				} else {
					switchDebounce++;
				}

				if (switchDebounce < SWITCHDEBOUNCE)
				{
					mState = CUT_ISO;
					motorTiming = cableEx.motorTimeout;
				} else {
					mState = CUTTING_ISO;
				}
				break;
			case CUTTING_ISO:

				if ( motorTiming == 0)
				{
					CUTTERREVERSE;
					mState = REVERSE;
				}
				break;
			case REVERSE:
				CUTTERREVERSE;
				mState = CUTTING;
				break;
			case CALIBRATE:
				mState = CUT;
				break;
			case CUTTING:

				if (CUTTERPOSSWITCH)
				{
					switchDebounce++;
				} else {
					switchDebounce = 0;
				}

				if (switchDebounce > SWITCHDEBOUNCE)
				{
					CUTTEROFF;
					mState = STOP;
				} else {
//					if (motorTiming == 0)
//					{
//						motorTiming = 200;
//					}
//					if (motorTiming >= 100)
//					{
//						PORTB |= ( 1 << PB2);
//					} else {
//						PORTB &= ( 1 << PB2);
//					}

					//PORTB ^= ( 1 << PB2);
				}
				break;



		}



		mainLoopCounter++;

	}
	return 0;
}
