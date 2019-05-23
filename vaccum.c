#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#define NR_PINS 3
#define RIGHT_FORWARD 1
#define RIGHT_BACK 0
#define LEFT_FORWARD 0
#define LEFT_BACK 1
#define RIGHT 1
#define LEFT 0

static volatile uint8_t count = 0;
static volatile uint8_t saved_tcnt0 = 0;
static volatile uint8_t saved_count = 0;
static volatile uint8_t triggers[3] = {PC0, PC2, PC4};
static volatile uint8_t echo_pins[3] = {PC1, PC3, PC5};
static volatile uint8_t flags[3] = {0};
static uint8_t front_sensor = 0, left_sensor = 0, right_sensor = 0;
static int my_rand;


ISR(TIMER0_OVF_vect, ISR_NOBLOCK)
{
	count++;
}

ISR(PCINT2_vect, ISR_NOBLOCK)
{
	if ((PINC & (1 << PC1)) != 0) {
		TCNT0 = 0;
		count = 0;
	}
	else {
		saved_tcnt0 = TCNT0;
		saved_count = count;
		flags[0] = 1;
	}
	
	if ((PINC & (1 << PC3)) != 0) {
		TCNT0 = 0;
		count = 0;
	}
	else {
		saved_tcnt0 = TCNT0;
		saved_count = count;
		flags[1] = 1;
	}
	
	if ((PINC & (1 << PC5)) != 0) {
		TCNT0 = 0;
		count = 0;
	}
	else {
		saved_tcnt0 = TCNT0;
		saved_count = count;
		flags[2] = 1;
	}
}

void HC_SR04_init()
{
	TCCR0A = 0;
	TCCR0B = 0;
	TIMSK0 = 0;

	sei(); // Enable global interrupts
	TCCR0B |= (1 << CS02); // Prescaler = 256 => T = 16us
	TIMSK0 |= (1 << TOIE0); // Enable Overflow interrupt

	
	for(int i = 0; i < NR_PINS; i++) {
		DDRC |= _BV(triggers[i]); // Trigger
		DDRC &= ~_BV(echo_pins[i]); // Echo
	}
	
	for(int i = 0; i < NR_PINS; i++) {
		PORTC &= ~(1 << triggers[i]);
	}

	/* Enable pin change interrupt on echo pin */
	PCICR |= _BV(PCIE2);
	PCMSK2 |= _BV(PCINT17) | _BV(PCINT19) | _BV(PCINT21) | _BV(PCINT23);
}

float HC_SR04_get_distance(int pin_position)
{
	double sum = 0;

	flags[pin_position] = 0;
	
	/* Reset Trigger signal for a clean rise */
	PORTC &= ~_BV(triggers[pin_position]);
	_delay_us(5);

	/* Send Trigger signal for > 10us */
	PORTC |= _BV(triggers[pin_position]);
	_delay_us(11);
	PORTC &= ~_BV(triggers[pin_position]);

	/* Wait for the Echo */
	while(flags[pin_position] == 0 && count <= 5);

	cli();
	sum = (((int)saved_tcnt0 + 255 * saved_count) * 16.0) * 0.01715;
	sei();

	return sum;
}

void right_MOTOR_set_duty_cycle(float percentage)
{
	if (percentage >= 0.0f && percentage <= 1.0f) {
		OCR2A = percentage * 256 - 1;

	} else {
		/* duty cycle 0% */
		OCR2A = -1;
	}
}

void left_MOTOR_set_duty_cycle(float percentage)
{
	if (percentage >= 0.0f && percentage <= 1.0f) {
		OCR2B = percentage * 256 - 1;

	} else {
		/* duty cycle 0% */
		OCR2B = -1;
	}
}

void MOTORS_init()
{
	TCCR2A = 0;
	/* Fast PWM mode 
	 * Top at 0xFF
	 */

    TCCR2A |= (3 << WGM20);
    /* Clear on comp OCR0A */
    TCCR2A |= (1 << COM2A1);
    /* Clear on comp OCR0B */
    TCCR2A |= (1 << COM2B1);
    /* prescaler 1*/
    TCCR2B |= (1 << CS20);

}

void set_direction_for_left(int a)
{
	
	if (a == 1) {

			PORTB &= ~(1 << PB4);
			PORTB |= (1 << PB5);
	
	} else {

			PORTB |= (1 << PB4);
			PORTB &= ~(1 << PB5);
	
	}

}

void set_direction_for_right(int a)
{
	
	if (a == 1) {

			PORTB &= ~(1 << PB6);
			PORTB |= (1 << PB7);
	
	} else {

			PORTB |= (1 << PB6);
			PORTB &= ~(1 << PB7);
	
	}

}

void stop_robot() {
    right_MOTOR_set_duty_cycle(0.0f);
    left_MOTOR_set_duty_cycle(0.0f);
}

void backwards_robot() {
	set_direction_for_right(RIGHT_BACK);
	set_direction_for_left(LEFT_BACK);
    right_MOTOR_set_duty_cycle(0.52f);
    left_MOTOR_set_duty_cycle(0.56f);
    _delay_ms(2000);
}

void straight_robot() {
	set_direction_for_right(RIGHT_FORWARD);
	set_direction_for_left(LEFT_FORWARD);
    right_MOTOR_set_duty_cycle(0.52f);
    left_MOTOR_set_duty_cycle(0.7f);
}

void set_speed_both_motors(float percentage) {
	right_MOTOR_set_duty_cycle(percentage);
	left_MOTOR_set_duty_cycle(percentage);
}

void turn_right_90_degrees()
{
	set_direction_for_right(RIGHT_BACK);
	set_direction_for_left(LEFT_FORWARD);
	right_MOTOR_set_duty_cycle(0.55f);
    left_MOTOR_set_duty_cycle(0.55f);
	_delay_ms(2000);
	stop_robot();
	_delay_ms(500);
}

void turn_left_90_degrees()
{
	set_direction_for_right(RIGHT_FORWARD);
	set_direction_for_left(LEFT_BACK);
	right_MOTOR_set_duty_cycle(0.55f);
    left_MOTOR_set_duty_cycle(0.55f);
	_delay_ms(2000);
	stop_robot();
	_delay_ms(500);
}

int main(void)
{
	double distance;
	double distance2;
	double distance3;

	srand(time(NULL));

	DDRA |= _BV(PA7);
	PORTA &= ~_BV(PA7);

	DDRA |= _BV(PA6);
	PORTA &= ~_BV(PA6);

	DDRA |= _BV(PA5);
	PORTA &= ~_BV(PA5);

	//pd6 si pd7 pentru duty cicle

	/* output for UP_MOTOR */
	DDRD |= (1 << PD6);
	PORTD &= ~(1 << PD6);

	/* output for DOWN_MOTOR */
	DDRD |= (1 << PD7);
	PORTD &= ~(1 << PD7);

	//pb4 si pb5 pentru a
	//pb6 si pb7 pentru motorul b

	DDRB |= (1 << PB4);
	DDRB |= (1 << PB5);
	DDRB |= (1 << PB6);
	DDRB |= (1 << PB7);

	MOTORS_init();
	HC_SR04_init();

	straight_robot();

	while(1) {
		/* 16MHz Timer freq, sound speed = 343 m/s */

		front_sensor = 0;
		right_sensor = 0;
		left_sensor = 0;

		//stanga
		distance = HC_SR04_get_distance(0);
		if (distance < 15) {
			PORTA |= _BV(PA5);
			left_sensor = 1;
		} else {
			PORTA &= ~_BV(PA5);
		}

		_delay_ms(100);

		//fata
		distance2 = HC_SR04_get_distance(1);
		if (distance2 < 15) {
			PORTA |= _BV(PA6);
			front_sensor = 1;
			stop_robot();
		} else {
			PORTA &= ~_BV(PA6);
			straight_robot();
		}

		_delay_ms(100);


		//dreapta
		distance3 = HC_SR04_get_distance(2);
		if (distance3 < 15) {
			PORTA |= _BV(PA7);
			right_sensor = 1;
		} else {
			PORTA &= ~_BV(PA7);
		}

		_delay_ms(100);

		if (front_sensor && left_sensor && right_sensor)
			backwards_robot();
		else if (front_sensor && right_sensor)
			turn_left_90_degrees();
		else if (front_sensor && left_sensor)
			turn_right_90_degrees();
		else if (front_sensor) {
			my_rand = rand() % 2;
			if (my_rand == RIGHT)
				turn_right_90_degrees();
			else
				turn_left_90_degrees();
		}

	}
}