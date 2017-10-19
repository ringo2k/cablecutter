#include <avr/io.h>
#include "lcd.h"
#include <avr/delay.h>
#include <avr/interrupt.h>

#define UPPRESSED (!(PINB & ( 1 << PB4)))
#define DOWNPRESSED !(PINB & ( 1 << PB5))
#define ENTERPRESSED !(PINC & ( 1 << PC3))
#define CUTTERPOSSWITCH !(PINC & ( 1 << PC2))

#define TOGGLELED (PORTB ^= ( 1 << PB0))

#define CUTTERON PORTC |= ( 1 << PC1)
#define CUTTEROFF PORTC &= ~ ( 1 << PC1)

typedef enum stepper_directions_m{
	RETRACT,
	EXTRACT
} stepperDirections;

struct cableExtruder
{
	uint16_t speed; //mm/s
	uint16_t length; //mm
	stepperDirections dir;
};

struct cableExtruder cableEx;
typedef enum motor_states_m{
	STOP,
	CUT,
	CUTTING,
	CALIBRATE
} motorState;

typedef enum cable_making_state{
	WAIT,
	START,
	EXTRUDING,
	CHOP,
	DONE
} cableMakingStates;


// this is for generating the steps
ISR(TIMER0_OVF_vect)
{
	static uint16_t tickCounter = 0;
	tickCounter++;

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

	}
    //PORTB ^= ( 1 << PB0);                    // Toggle PB0
                                      // f_PB0 = 1MHz/1042/256/2 = 1.9Hz
}

int main()
{
	unsigned int mainLoopCounter = 0;
	motorState mState = STOP;
	cableMakingStates makingState = WAIT;

	cableEx.speed = 200;
	cableEx.length =0;
	cableEx.dir = EXTRACT;

	//IO
	DDRB |= ( 1 << PB0); //LED
	DDRD |= ( 1 << PD6); //DIR
	DDRD |= ( 1 << PD7); //Step

	PORTB |= ( 1 << PB4); // Button up
	PORTB |= ( 1 << PB5); // Button down
	PORTC |= ( 1 << PC3); // Button enter
	PORTC |= ( 1 << PC2); // PU for Cutter Pos switch

	DDRC |= (( 1 << PC1));

	//timer init
	TCCR0 |= ( 1 << CS01); // Prescaling 8
	TIMSK |= ( 1 << TOIE0); // Interrupt enable
	sei();

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

		if (!( mainLoopCounter % 8))
		{
			//PORTB ^= ( 1 << PB0);

			//PORTD ^= ( 1 << PD7); //Step

		}

		if (!( mainLoopCounter % 100))
		{
			lcd_clrscr();
			//show motor state in first line
			lcd_puts("CUTTER: ");
			switch (mState)
			{
			case STOP:
				lcd_puts("STOP");
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
				case EXTRUDING:
					lcd_puts("EXTRUDING");
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

			mState = CUT;


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
					cableEx.length = 200;
					makingState=EXTRUDING;
				}
				break;
			case EXTRUDING:
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
					mState = CUT;
				} else {
					mState = CUTTING;
				}
				break;
			case CALIBRATE:
				mState = CUT;
				break;
			case CUTTING:
				if (CUTTERPOSSWITCH)
				{
					mState = STOP;
				}
				break;



		}



		mainLoopCounter++;

	}
	return 0;
}
