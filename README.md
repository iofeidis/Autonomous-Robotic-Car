# Autonomous-Robotic-Car

/*------Autonomous Robotic Car using Ultrasound Sensor SRF04------*/

**Authors: Iason Ofeidis, Simonas Kundrotas
**Subject: Microprocessor Techniques and Embedded Systems (CMPT 18/19Z)
**Professor: Tomas Fryza
 
/*-----Source File Description--------*/

1. Headers

#include <avr/io.h>
#include <MrLcd/MrLCDmega32.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

2. Defining variables

static volatile int pulse = 0;
static volatile int i = 0;

 ** The variable ‘pulse’ is used to store the count value from the TCNT register.
 ** The variable ‘i’ is used as a flag to indicate the current status of the Echo pin.

3. The Interrupt Service Routine

**ISR stands for interrupt service routine. It is a function that is executed when the 
**microcontroller is interrupted.

ISR(INT0_vect)
{
  if(i == 1)
  {
    TCCR1B = 0;
    pulse = TCNT1;
    TCNT1 = 0;
    i = 0;
  }
  if(i == 0)
  {
    TCCR1B |= 1<<CS10;
    i = 1;
  }
}

**The echo pin is the one that becomes high after it receives reflected waves. The time for 
**which the echo pin is high is directly proportional to the distance of the obstacle from sensor.

**Thus we need to calculate the time for which this pin has stayed high. For this we use the 
**internal counter of the microcontroller.

A. When Echo Pin goes from low to high (0 to 1)

**We have previously defined a variable ‘i’ that had initial value 0. When the echo PIN goes high 
**the controller is interrupted and the ISR is executed. The condition is checked with the ‘if’
**statement and the microcontroller starts the counter and also sets the value of ‘i’ to 1.

**TCCR stands for TIMER COUNTER CONTROL REGISTER. Setting its CS02 and CS00 bits to 1 starts
**the timer with a prescaling of 1024. The count value is stored in a register called TCNT.

B. When Echo pin goes form high to low (1 to 0)

**When the echo pin goes low, microcontroller is interrupted again and again the ISR is executed.
**This time value of ‘i’ is 1 (We had changed it from 0 to 1 when we had started the timer).

**The ‘if’ condition will be checked again. This time the timer will be stopped by setting TCCR
**to zero.The value that was counted will be saved in TCNT register. We will store that value in
**a previously defined variable ‘pulse’ and clear/ reset the value of TCNT.

4. setup() 

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

**Set pins to HIGH, so the motors and wheels are ready to move

5. motorAforward(), motorBforward, motorAbackward(), motorBbackward

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

**Depending on which pins are ON and OFF, each side of the motor (A or B) moves respectively.

6. forward(), backward(), stop(), right()

/* Car moves forward */
void forward () {
 motorAforward();
 motorBforward();
}

/* Car moves backward */
void backward()  {
 motorAbackward();
 motorBbackward();
}

/* Stop the Car */
void stop(){
 PORTB |= _BV(PB1);       // PORTB1 = HIGH
 PORTB |= _BV(PB2);       // PORTB2 = HIGH
 PORTB |= _BV(PB4);       // PORTB4 = HIGH
 PORTD |= _BV(PD5);       // PORTD5 = HIGH
}

/* Car turns right */
void right() {
 motorAbackward();
 motorBforward();
}

**Same as before, we modify the pins' state (HIGH/LOW) to make various car movements.

7. int main() /*---Main Function---*/

setup();

  PORTB |= _BV(PB3);
  PORTB |= _BV(PB0);

**Initializations for the motor pins

  int16_t count_a = 0;  // Distance in cm from object
**Variable for storing our timer value 
 
  DDRD |= _BV(PD0);     // PIND0 = OUTPUT
**PIND0 is where sensor's Trigger is connected

  DDRD &= ~_BV(PD2);    // PIND2 = INPUT, INT0
**PIND2 is where sensor's Echo is connected

  /* External Interrupt: enable External Interrupt by INT0 pin 2 (PD2) */
  /* Any logical change of INT0 generates an interrupt request */
  EICRA |= (~_BV(ISC01)) | (_BV(ISC00));
**Prescaler value of 1024
 
  /* Enable External Interrupt Request 0 */
  EIMSK |= _BV(INT0);


  sei();                    //Enable Interrupts

/*----Main Loop-------*/

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


/*------ End of Description ------*/
