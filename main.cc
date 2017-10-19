#include <avr/io.h>
#include "lcd.h"
#include <avr/delay.h>

#define UPPRESSED (!(PINB & ( 1 << PB4)))
#define DOWNPRESSED !(PINB & ( 1 << PB5))
#define ENTERPRESSED !(PINC & ( 1 << PC3))


int main()
{
	unsigned int mainLoopCounter = 0;

	//IO
	DDRB |= ( 1 << PB0); //LED
	DDRD |= ( 1 << PD6); //DIR
	DDRD |= ( 1 << PD7); //Step

	PORTB |= ( 1 << PB4); // Button up
	PORTB |= ( 1 << PB5); // Button down
	PORTC |= ( 1 << PC3); // Button enter

	DDRC |= (( 1 << PC1) | ( 1 << PC2));


	//UI
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_puts("Hello World!");
	lcd_puts("\n Works!");

	while(1)
	{

		if (!( mainLoopCounter % 8))
		{
			PORTB ^= ( 1 << PB0);
			//PORTD ^= ( 1 << PD7); //Step

		}

		if (!( mainLoopCounter % 10000))
		{
			//PORTB ^= ( 1 << PB0);
			PORTD ^= ( 1 << PD6); //DIR

		}

		if (UPPRESSED)
		{
			lcd_clrscr();
			lcd_puts("UP");
			while(UPPRESSED);

			//activate Motor
			PORTC |= ( 1 << PC1);
			_delay_ms(200);
			PORTC &= ~(( 1 << PC1) | ( 1 << PC2));
			_delay_ms(200);



		}

		if (DOWNPRESSED)
		{
			lcd_clrscr();
			lcd_puts("DOWN");
			while(DOWNPRESSED);
			//activate Motor
			PORTC |= ( 1 << PC2);
			_delay_ms(200);
			PORTC &= ~(( 1 << PC1) | ( 1 << PC2));
			_delay_ms(200);

		}

		if (!ENTERPRESSED)
		{
			lcd_clrscr();
			lcd_puts("ENTER");
			while(!ENTERPRESSED);

		}


		mainLoopCounter++;

	}
	return 0;
}
