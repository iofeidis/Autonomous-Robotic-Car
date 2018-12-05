/* START OF FILE */

/**
 * @Autonomous Robotic Car using Ultrasound Sensor SRF04
 * @Authors: Iason Ofeidis, Simonas Kundrotas
 * @Subject: Microprocessor Techniques and Embedded Systems (CMPT 18/19Z)
 * @Professor: Tomas Fryza
 * @Date:   2018-11-29T14:21:56+01:00
 * @Last modified time: 2018-11-29T14:21:56+01:00
 */

/* Measuring distance using ultrasonic distance sensor */
#include "settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

/*volatile for compiler optimization */
static volatile int pulse = 0;  //duration of sensor pulse echo
static volatile int i = 0;      //flag for interrupt


/* Any logical change in PIND2/INT0 triggers the ISR */
ISR(INT0_vect)
{
  if(i == 1)            // Sensor receives echo pulse
  {

    TCCR1B = 0;         // Turns the timer off
    pulse = TCNT1;      // Stores timer value into variable pulse
    TCNT1 = 0;          // Resets timer value
    i = 0;              // Flag = 0
  }

  if(i==0)              // Sensor transmits pulse
  {
    //PORTB ^= _BV(PB5);                // Debugging
    TCCR1B |= _BV(CS02) | _BV(CS00);    // Prescaler 1024 (overflow 16msec)
    i = 1;                              // Flag = 1
  }
}

/* Main Function */
int main(void)
{

  setup();

  PORTB |= _BV(PB3);
  PORTB |= _BV(PB0);

  int16_t count_a = 0;  // Distance in cm from object
  //DDRD = 0b11111011;
  DDRD |= _BV(PD0);     // PIND0 = OUTPUT

  DDRD &= ~_BV(PD2);    // PIND2 = INPUT, INT0


  /* Debugging */
  // DDRB |= _BV(PB5);      // PINB5 = OUTPUT
  // PORTB &= ~_BV(PB5);    // PINB5 = LED ON (HIGH)


  /* External Interrupt: enable External Interrupt by INT0 pin 2 (PD2) */
  /* Any logical change of INT0 generates an interrupt request */
  EICRA |= (~_BV(ISC01)) | (_BV(ISC00));
  /* Enable External Interrupt Request 0 */
  EIMSK |= _BV(INT0);


  sei();                    //Enable Interrupts

  /* Main Loop */
  while(1)
  {

    /* Trigger the sensor */
    /* Sending a pulse with duration 15 usec */

    PORTD |= _BV(PD0);      // PIND0 = HIGH

    _delay_us(15);          // Duration of pulse == 15usec

    PORTD &= ~_BV(PD0);     // PIND0 = LOW

    /* Measure the distance by the duration of the pulse */

    count_a = pulse;        // Distance from object in cm

    /* Condition regarding the distance from object */


    if (count_a < 25) {

        /* Object is too close to the car (distance < 25 cm)

        /* Debugging */
        // PORTB &= ~_BV(PB5);

        stop();                 // Stop the car
        _delay_ms(300);         // Wait 300 msec
        backward();             // Go backwards for 500 msec
        _delay_ms(500);

        stop();                 // Stop the car
        _delay_ms(300);         // Wait 300 msec
        right();                // Turn right for 700 msec
        _delay_ms(700);
        stop();                 // Stop the car
        _delay_ms(300);         // Wait 300 msec

    } else {

        /* There is no object near the car with distance <25 cm

        /* Debugging */
        // PORTB |= _BV(PB5);

        forward();              // Go forward for 200 msec
        _delay_ms(200);

    }
  }
}

/* Setup the Car Motors */
void setup(void)
{

  /* Set ports PB1:4 and PD5 for OUTPUT */
  DDRB |= _BV(PB1);
  DDRB |= _BV(PB2);
  DDRB |= _BV(PB2);
  DDRB |= _BV(PB3);
  DDRB |= _BV(PB4);
  DDRD |= _BV(PD5);
}

/* A side of the Car moves forward */
void motorAforward() {
  PORTB |= _BV(PB1);      // PORTB1 = HIGH
  PORTB &= ~_BV(PB2);     // PORTB2 = LOW

}

/* B side of the Car moves forward */
void motorBforward() {
  PORTB |= _BV(PB4);      // PORTB4 = HIGH
  PORTD &= ~_BV(PD5);     // PORTD5 = LOW
}

/* Car moves forward */
void forward () {
 motorAforward();
 motorBforward();
}

/* A side of the Car moves backward */
void motorAbackward() {
  PORTB &= ~_BV(PB1);     // PORTB1 = LOW
  PORTB |= _BV(PB2);      // PORTB2 = HIGH
}

/* B side of the Car moves backward */
void motorBbackward() {
  PORTB &= ~_BV(PB4);
  PORTD |= _BV(PD5);
}

/* Stop the Car */
void stop(){
 PORTB |= _BV(PB1);       // PORTB1 = HIGH
 PORTB |= _BV(PB2);       // PORTB2 = HIGH
 PORTB |= _BV(PB4);       // PORTB4 = HIGH
 PORTD |= _BV(PD5);       // PORTD5 = HIGH
}

/* Car moves backward */
void backward()  {
 motorAbackward();
 motorBbackward();
}

/* Car turns right */
void right() {
 motorAbackward();
 motorBforward();
}

/* Debugging */
/*
void avoid(){
 stop();
_delay_ms(500);
   backward();
    _delay_ms(2800);
stop();
_delay_ms(500);
  right();
  _delay_ms(20);
}
*/


/* END OF FILE */
