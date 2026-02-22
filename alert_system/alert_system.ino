/*

	Example of use of the FFT library to compute FFT for a signal sampled through the ADC.
  
  Copyright (C) 2018 Enrique Condés and Ragnar Ranøyen Homb
  Copyright (C) 2020 Bim Overbohm (template, speed improvements)

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "arduinoFFT.h"

/*
These values can be changed in order to evaluate the functions
*/
#define LED 13
#define CHANNEL A0
#define MOTOR 9
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 8000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{

  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Ready");
  pinMode(LED, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  digitalWrite(MOTOR, LOW);
}

void loop()
{
  /*SAMPLING*/
  microseconds = micros();

  // Takes 16ms per sample
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
  FFT.complexToMagnitude(); /* Compute magnitudes */
  Serial.print("3000HZ ");
  Serial.println(vReal[48], 4);

  if (vReal[48] > 750) {
    flashLED();
  }
}

void flashLED() {
  int ledState = LOW;
  digitalWrite(MOTOR, HIGH);

  // Toggle LED 8 times
  for (int i = 0; i < 8; i++) {
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(LED, ledState);
    delay(250);
  }  
  digitalWrite(MOTOR, LOW);
}