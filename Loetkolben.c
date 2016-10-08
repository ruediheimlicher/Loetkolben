//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//


#include <stdlib.h>

//#include <avr/io.h>

#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#include <avr/wdt.h>

//#include "lcd.c"

#include "adc.c"
#include "defines.h"

volatile    	uint16_t loopcount0=0;
volatile       uint16_t loopcount1=0;

volatile    uint16_t timercount0=0;
volatile    uint16_t timercount1=0;
volatile    uint16_t adccount0=0;
volatile    uint8_t blinkcount=0;

volatile    uint8_t pwmpos=0;
volatile    uint8_t grenzwert=0;
volatile    uint8_t istwert=0;
volatile    uint8_t lastwert=0;
volatile    int16_t fehler=0;
volatile    int16_t lastfehler=0;
volatile    int16_t fehlersumme=0;

volatile    double stellwert=200.0;
volatile    uint8_t status=0;

volatile    uint8_t switchdebounce=0;
volatile    uint16_t switchdelay=0;


void delay_ms(unsigned int ms);


void slaveinit(void)
{
 	OUTDDR |= (1<<PWM_OUT_PIN);		//Pin 0 von PORT D als Ausgang fuer PWM
	OUTPORT |= (1<<PWM_OUT_PIN);		//HI

   OUTDDR &= ~(1<<SWITCH_PIN);		//Eingang fuer Switch
   OUTPORT |= (1<<SWITCH_PIN);		//HI

   OSZIDDR |= (1<<OSZIA);		//Pin 1 von PORT D als Ausgang fuer OSZI
 	OSZIPORT |= (1<<OSZIA);		//HI
   //DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
   LOOPLEDDDR |= (1<<LOOPLED_PIN);		//Pin 4 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED_PIN);		//Pin 4 von PORT D als Ausgang fuer LED Loop

   LOOPLEDDDR |= (1<<TOPLED_PIN);		//Pin 5 von PORT D als Ausgang fuer LED Heizung
	
   
   ADCDDR &= ~(1<<ADC_RED_PIN);	//als Eingang fuer ADC PWM-Wert
   
   //GIMSK |= (1<<INT0); // Interrupt en
   
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

#pragma mark Takt
void timer0(void) //Takt der Messung
{
	//----------------------------------------------------
	// Set up timer 0
	//----------------------------------------------------
   /*
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS00) | _BV(CS02);
    OCR0A = 0x2;
    TIMSK0 = _BV(OCIE0A);
    */
   
//   DDRD |= (1<< PORTD6);   // OC0A Output
   /*
   TCCR0A |= (1<<WGM00);   // fast PWM  top = 0xff
   TCCR0A |= (1<<WGM01);   // PWM
   //TCCR0A |= (1<<WGM02);   // PWM
   
   TCCR0A |= (1<<COM0A1);   // set OC0A at bottom, clear OC0A on compare match
   TCCR0B |= 1<<CS02;
   TCCR0B |= 1<<CS00;
   
   OCR0A=10;
   TIMSK0 |= (1<<OCIE0A);
   */
   
   TCCR0B |= (1<<CS00)|(1<<CS01);	//Takt /64
   
   TCCR0A |= (1<<WGM01);   // Enable  compare match
   
   
  // TCCR0B |= (1<<CS02);	//Takt /256 Intervall
   
   //TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
    TIMSK |= (1<<OCIE0A);			//Overflow Interrupt aktivieren
   
   TCNT0 = 0x00;					//Rücksetzen des Timers
   
   OCR0A = TIMER2_COMPA; // Compare match A
   TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts

}


ISR(TIMER0_COMPA_vect)
{
   //LOOPLEDPORT ^= (1<<LOOPLED_PIN);
   timercount0++;
   if (timercount0 > 4) // Takt teilen, 1s
   {
      //OSZITOGG;
      //LOOPLEDPORT ^= (1<<TOPLED_PIN);
      timercount0=0;
      
      timercount1++;

      uint8_t diff = (timercount1 - grenzwert);
      
      // Leistung reduzieren wenn Loetkolben in Ablage
      if ((timercount1 >= grenzwert) && (status & (1<<SWITCH_ON)) && grenzwert) // grenzwert erreicht
      {
         //OSZIHI;
        // OSZILO;
         //LOOPLEDPORT &= ~(1<<LOOPLED_PIN);
         OUTPORT &= ~(1<<PWM_OUT_PIN);    // Mosfet off
         LOOPLEDPORT &= ~(1<<TOPLED_PIN);
         // status |= (1<<PWM_ADC);// ADC messen ausloesen
      }
      
      
      
      // Takt
      if (timercount1 == TIMER2_PWM_INTERVALL) // Paketbreite
      {
         

        // OSZITOGG;
         //OSZIHI;
         timercount1 = 0;
         
         status |= (1<<PWM_ADC);// ADC messen ausloesen
         //LOOPLEDPORT |= (1<<LOOPLED_PIN);
         LOOPLEDPORT |= (1<<TOPLED_PIN);
         OUTPORT |= (1<<PWM_OUT_PIN); // Mosfet on
         
      }
      
   }
   
}

ISR(TIMER0_OVF_vect)
{
   
   //LOOPLEDPORT |=(1<<LOOPLED);
}
#pragma mark // Timer1 PWM
// Timer1 fuer PWM
void timer1(void)
{
  // TCCR0A = 0;  // normal mode
 //  TCCR0B = 0;
   TCCR1 = 0;                  //stop the timer
   TCNT1 = 0;                  //zero the timer
   GTCCR = _BV(PSR1);          //reset the prescaler
   OCR1A = 100;                //set the compare value
   OCR1C = 255;
   TIMSK |=(1<<OCIE1A);        //interrupt on Compare Match A
   TIMSK |=(1<<TOIE1);
   //start timer, ctc mode, prescaler clk/16384
  // TCCR1 |= (1 << CTC1);
   TCCR1 |= (1<<CS10   | 1<<CS12 | 1<<CS11 );
   sei();

}

// Timer1 fuer PWM: Interrupt bei OCR1A, in
ISR(TIMER1_COMPA_vect)
{
   //comment out one of the two lines below
   //digitalWrite(4, LOW);       //turn the LED off
   //PINB |= _BV(PINB4);         //flash the LED by toggling PB4
   //LOOPLEDPORT &= ~(1<<LOOPLED);
   OUTPORT &= ~(1<<TOPLED_PIN); // LED off off

}

ISR(TIMER1_OVF_vect)
{
  // LOOPLEDPORT |= (1<<LOOPLED);
 //  if (stellwert>10)
   {
      OUTPORT |= (1<<TOPLED_PIN); // LED on
   }
}

#pragma mark INTO
ISR(INT0_vect) // Eingang auf 0
{
//   status |= (1<<SWITCH_ON); // Loetkolben ist in Ablage
 //  switchdebounce++;
}

int main (void)
{
   MCUSR = 0;
	//wdt_disable();

	slaveinit();
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	
	//uint16_t ADC_Wert= readKanal(0);
		
	/* initialize the LCD */
//	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

//	lcd_puts("Guten Tag\0");
//	delay_ms(1000);
//	lcd_cls();
//	lcd_puts("READY\0");
	
	
	//initADC(TASTATURPIN);
	
	//uint16_t startdelay1=0;

	//uint8_t twierrcount=0;
	//LOOPLEDPORT |=(1<<LOOPLED);
	
	//delay_ms(800);

//	lcd_clr_line(0);
   timer0();
//   timer1(); // PWM
   initADC(0);
//   grenzwert = readKanal(2)>>2;
   sei();
	while (1)
   {
      //Blinkanzeige
      loopcount0++;
      if (loopcount0 >= LOOPSTEP)
      {
         
         //grenzwert = 88;
         //OCR1A = istwert;

         loopcount0=0;
         // LOOPLEDPORT ^=(1<<LOOPLED);
         loopcount1++;
         //OSZITOG;
         if (loopcount1 >= LOOPSTEP)
         {
            //LOOPLEDPORT ^= (1<<TOPLED_PIN);
           // LOOPLEDPORT ^= (1<<PORTB1);
            loopcount1 = 0;
          //  LOOPLEDPORT ^=(1<<LOOPLED);
            /*
            if (grenzwert > istwert)
            {
               LOOPLEDPORT ^=(1<<TOPLED_PIN); // TEMPERATUR NOCH ZU KLEIN
            }
            else
            {
               LOOPLEDPORT |=(1<<TOPLED_PIN);
            }
             */
         }
      }
  
      /*
      if (OUTPIN & (1<<SWITCH_PIN)) // Schalter offen, Loetkolben weg
      {
         OUTPORT |= (1<<TOPLED_PIN);
      }
      else
      {
         OUTPORT &= ~(1<<TOPLED_PIN);
      }
      */
      if (status & (1<<PWM_ADC)) // Takt 1s
      {
         //OSZILO;
         status &= ~(1<<PWM_ADC);
         //OSZITOGG;
         // Grenzwert lesen
         grenzwert = readKanal(3)>>2;
                  //Schalterstellung abfragen
         
         if (OUTPIN & (1<<SWITCH_PIN)) // Schalter offen, Loetkolben weg, heizen voll
         {
            OUTPORT ^= (1<<TOPLED_PIN);
            //OSZILO;
            status &= ~(1<<SWITCH_ON);
            switchdelay = 0;
            OUTPORT |= (1<<PWM_OUT_PIN); // Triac on
            LOOPLEDPORT |= (1<<TOPLED_PIN);
            
         }
         else if (!(status & (1<<SWITCH_ON))) // Loetkolben neu in Ablage,
         {
            
            switchdelay++; // Zeit messen
            if (switchdelay > SWITCH_DELAY) // nach delay Leistung reduzieren
            {
               //OSZILO;
               status |= (1<<SWITCH_ON); // Loetkolben ist in Ablage
               switchdelay = 0;
            }
            
         }
         
         // in 328 Kanaele vertauscht
  //       grenzwert = readKanal(2)>>2;
  //       istwert = readKanal(1)>>2;

#pragma mark PID
         /*
          e = w - x;                       //Vergleich
          esum = esum + e;                 //Integration I-Anteil
          if (esum < -400) {esum = -400;}  //Begrenzung I-Anteil
          if (esum > 400) {esum = 400;}
          y = Kp*e + Ki*Ta*esum;           //Reglergleichung
          //y = Kp*e + Ki*Ta*esum + Kd/Ta*(e – ealt);   //Reglergleichung
          if (y < 0) 
          {
          y = 0;
          }              //Begrenzung Stellgröße
          
          
          if (y > 255) {y = 255;}
          
          PWM = y;                         //Übergabe Stellgröße
          
          
          int8_t fehler=0;
          int16_t fehlersumme=0;
          Führungsgröße (Sollwert) w: Vorgegebener Wert, auf dem die Regelgröße durch die Regelung gehalten werden soll. Sie ist eine von der Regelung nicht beeinflusste Größe und wird von außen zugeführt.
          Regelgröße (Istwert) x
          */
         /*
         if (istwert < grenzwert/4*3)
         {
            status |= (1<<PID_FIRST_RUN); // K Prop ist beim ersten Aufheizen kleiner
            
         }

        
         fehler = grenzwert - istwert; // Fehler positiv wenn temp zu klein
         
         fehlersumme += fehler;
         
         if (fehlersumme < K_WINDUP_LO)
         {
            fehlersumme = K_WINDUP_LO;
         }
         if (fehlersumme > K_WINDUP_HI)
         {
            fehlersumme = K_WINDUP_HI;
         }
         
         
         if (fehlersumme>=0)
         {
//            lcd_putc(' '); // Platz des Minuszeichens
         }
         
#pragma mark stellwert
 
         
         float k_prop = K_PROP_HI;
         if (status & (1<<PID_FIRST_RUN))
         {
            k_prop = K_PROP_LO;
            
            if (istwert > grenzwert) // zuruecksetzen wenn soll erreicht
            {
               status &= ~(1<<PID_FIRST_RUN);
               k_prop = K_PROP_HI;
            }
         }
         
         
         stellwert = k_prop * fehler + K_INT * K_DELTA * fehlersumme  + K_DIFF*((fehler-lastfehler) /K_DELTA);

         stellwert *= PWM_FAKTOR;
         //stellwert = 83;
         
         lastfehler = fehler;
         
         
         if (stellwert < 0)
         {
            stellwert = 0;
            //OUTPORT &= ~(1<<PWM_OUT_PIN); // Triac off
         }
         if (stellwert == 0)
         {
            OUTPORT &= ~(1<<PWM_OUT_PIN); // Triac sicher off
         }
         
         if (stellwert > 254)
         {
            stellwert = 254;
         }
         */
   //      OCR1A = (uint16_t)stellwert;
  
 
      
      }
      
      
      
      
      
   
      //	LOOPLEDPORT &= ~(1<<LOOPLED);
   }//while


 return 0;
}
