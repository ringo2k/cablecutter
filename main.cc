#include <avr/io.h>
#include "lcd.h"
#include <avr/delay.h>
#include <avr/interrupt.h>

#define UPPRESSED (!(PINB & ( 1 << PB4)))
#define DOWNPRESSED !(PINB & ( 1 << PB5))
#define ENTERPRESSED !(PINC & ( 1 << PC3))
#define CUTTERPOSSWITCH !(PINC & ( 1 << PC2))

#define TOGGLELED (PORTB ^= ( 1 << PB0))

#define CUTTERON PORTC &= ~( 1 << PC1); PORTB |= ( 1 << PB2)
#define CUTTEROFF PORTB |= ( 1 << PB2);PORTC |= ( 1 << PC1)
#define CUTTERREVERSE PORTC |= ( 1 << PC1); PORTB &= ~( 1 << PB2)

#define STEPPER_ON PORTC |= ( 1 << PC5)
#define STEPPER_OFF PORTC &= ~(1 << PC5)

#define MMPERSTEP 0.22
#define SWITCHDEBOUNCE 100

uint32_t motorTiming = 0;


typedef enum stepper_directions_m{
	RETRACT,
	EXTRACT
} stepperDirections;

struct cableExtruder
{
	uint16_t speed; //mm/s
	uint32_t length; //mm
	float unisolatedLength; // mm
	float targetLength;

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

int main()
{
	unsigned int mainLoopCounter = 0;
	unsigned int switchDebounce = 0;
	motorState mState = STOP;
	cableMakingStates makingState = WAIT;

	cableEx.speed = 40;
	cableEx.length =0;
	cableEx.dir = EXTRACT;
	cableEx.unisolatedLength = 3.0;
	cableEx.targetLength = 40.0;

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

    OCR1B = 0;
	//timer init stepper
	TCCR0 |= ( 1 << CS01); // Prescaling 8
	TIMSK |= ( 1 << TOIE0); // Interrupt enable
	sei();

	while(0)
	{
		PORTC |= ( 1 << PC1);

		PORTB ^= ( 1 << PB2);
		_delay_ms(10);
		if (CUTTERPOSSWITCH)
		{
			PORTC &= ~( 1 << PC1);
			PORTB &= ~( 1 << PB2);
			_delay_ms(5000);
			PORTB |= ( 1 << PB2);
		}
	}


	if (CUTTERPOSSWITCH)
	{
		mState = STOP;
	} else {
		mState = CALIBRATE;
	}

	//UI
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_puts("Hello World!");
	lcd_puts("\n Works!");

	while(1)
	{


		if (!( mainLoopCounter % 4000))
		{
			lcd_clrscr();
			//show motor state in first line
			lcd_puts("CUTTER: ");
			switch (mState)
			{
			case STOP:
				lcd_puts("STOP");
				break;
			case CUT_ISO:
				lcd_puts("ISO");
				break;
			case REVERSE:
				lcd_puts("REV");
				break;
			case CUT:
				lcd_puts("CUT");
				break;
			case CUTTING:
				lcd_puts("CUT*");
				break;
			case CALIBRATE:
				lcd_puts("CAL");
				break;

			default:
				lcd_puts("unknown");
				break;

			}
			lcd_puts("\n");
			switch (makingState) {
				case WAIT:
					lcd_puts("WAITING");
					break;
				case EXTRUDING_MAIN:
					lcd_puts("EXT_MAIN");
					break;
				case EXTRUDING_BACK:
					lcd_puts("EXT_BACK");
					break;
				case EXTRUDING_FRONT:
					lcd_puts("EXT_FRONT");
					break;
				case CHOP:
					lcd_puts("CHOP");
					break;
				case DONE:
					lcd_puts("FINISH");
					break;

				default:
					break;
			}

			if (cableEx.length > 0)
			{
				lcd_puts(" +");

			}else {
				lcd_puts(" -");
			}
			//PORTB ^= ( 1 << PB0);
			//PORTD ^= ( 1 << PD6); //DIR

		}

		if (UPPRESSED)
		{
			lcd_clrscr();
			lcd_puts("UP");
			while(UPPRESSED);

			mState = CUT_ISO;


		}

//		if (CUTTERPOSSWITCH)
//		{
//			lcd_clrscr();
//			lcd_puts("pushed");
//		} else {
//			lcd_clrscr();
//			lcd_puts("not pushed");
//		}


		if (!ENTERPRESSED)
		{
			lcd_clrscr();
			lcd_puts("ENTER");
			while(!ENTERPRESSED);

		}

		switch (makingState) {
			case WAIT:
				// Switch activation
				if (DOWNPRESSED)
				{
					while(DOWNPRESSED);
					STEPPER_ON;
					_delay_ms(100);
					cableEx.length = (uint32_t)(cableEx.unisolatedLength / MMPERSTEP);
					makingState=EXTRUDING_FRONT;
				}
				break;

			case EXTRUDING_FRONT:
				if ( cableEx.length == 0)
				{
					mState = CUT_ISO;
					makingState = RETRACT_FRONT;
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
				makingState = WAIT;
				STEPPER_OFF;
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
					motorTiming = 1510;
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
