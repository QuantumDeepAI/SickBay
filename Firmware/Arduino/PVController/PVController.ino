#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <arduinoFFT.h>
#include <simpleRPC.h>
#include <Adafruit_BMP280.h>

#define SPO2Sensor1Pin A0

#define DEBUG_USE_FFT true  // temporary until FFT implementation and options are decided

enum FFT_SCALE {
  FFT_SCALE_INDEX,
  FFT_SCALE_TIME,
  FFT_SCALE_FREQUENCY,
  FFT_SCALE_PLOT
};

/*
 * 
 * This file is a starting point but needs a lot of decisive thought and cleanup
 * 
 * 
   pseudocode planning

   basic check in test functionality: read sensors and report over serial 

   clean configuration of sensors - some sensors need quite a library adding sensors is detailed do by hand

   make a list(hardcoded but failable) of sensors
   report sensor data in a loop continuously
      or pull sensor data when requested - serial shell /RPC

   serial console to set values other than default, to override servo function?
   try to use uniform name casing with libraries that choose different methods of naming. 

   report format:
    send csv header with field names
    <field 1>, <field 2>, <field 3>, <field 4>
    #,#,#,,\n    (,, sensor didn't initialize properly so its not being reported in the csv as a value *kind of NULL* )
    #, E:003:<message>, 3
    listen for commands to set or overrride any configurable properties or values - since this device requires a display and computer it can be set on boot. we won't be saving values in eeprom
    error codes as numbers
*/
// set up variable to print/plot this as a debug mode vs report stats

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *TemperatureSensor1 = bmp.getTemperatureSensor();
Adafruit_Sensor *PressureSensor1 = bmp.getPressureSensor();

Servo VentilatorServo1;

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 9;

unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[samples];
double vImag[samples];

void setup() {

  Serial.begin(115200);

  arduinoFFT FFT = arduinoFFT();
  
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  TemperatureSensor1->printSensorDetails();
    
  VentilatorServo1.attach(9);
  
  Serial.println("SPO2Sensor1, TemperatureSensor1, PressureSensor1, Servo1 Position");
}

void WriteVentilatorServo1(int position) {
  return VentilatorServo1.write(position);
}

void ReadTemperatureSensor1() {
  sensors_event_t temp_event;
  return TemperatureSensor1->getEvent(&temp_event);
}

void ReadPressureSensor1() {
  sensors_event_t pressure_event;
  return PressureSensor1->getEvent(&pressure_event); 
}

void loop() {

  interface(
    digitalRead,
      "digital_read: Read digital pin. @pin: Pin number. @return: Pin value.",
    digitalWrite,
      "digital_write: Write to a digital pin. @pin: Pin number. @value: Pin value.");

  /* DS100A SPO2 Sensor read loop */
  Serial.print(F("SP02 reading = "));
  Serial.println(analogRead(A0));

  /* BMP280 Temperature and Pressure Sensor read loop */
  sensors_event_t temp_event, pressure_event;

  TemperatureSensor1->getEvent(&temp_event);
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  PressureSensor1->getEvent(&pressure_event);
  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.print(F("Ventilator Servo Position = "));
  Serial.println(VentilatorServo1.read());
  //delay(25);

  //Serial.println(sprintf("%f, %f, %f", analogRead(A0), analogRead(A1), analogRead(A2)));
  //Serial.printf("%s, %s, %s", inA0, inA1, inA2);
  //"%s, %s, %s\n", inA0, inA1, inA2

  //Serial.println();
  //delay(2000);
 
  /* DS100A SPO2 Sensor FFT calculations */
  
  /*
  microseconds = micros();
  for (int i = 0; i < samples; i++)
  {
    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    //while(micros() - microseconds < sampling_period_us){
    //empty loop
    //}
    //microseconds += sampling_period_us;
  }*/
  /* Build raw data */
  //  double cycles = (((samples-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
  //  for (uint16_t i = 0; i < samples; i++)
  //  {
  //    vReal[i] = int8_t((amplitude * (sin((i * (twoPi * cycles)) / samples))) / 2.0);/* Build data with positive and negative values*/
  //    //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
  //    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  //  }
  /* Print the results of the simulated sampling according to time */
  //Serial.println("Data:");
  //PrintVector(vReal, samples, FFT_SCALE_TIME);
  // FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //Serial.println("Weighed data:");
  //PrintVector(vReal, samples, FFT_SCALE_TIME);
  //FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //Serial.println("Computed Real values:");
  //PrintVector(vReal, samples, FFT_SCALE_INDEX);
  //Serial.println("Computed Imaginary values:");
  //PrintVector(vImag, samples, FFT_SCALE_INDEX);
  //FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  // Serial.println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), FFT_SCALE_FREQUENCY);
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //Serial.println(x, 6);
  //while(1); /* Run Once */
  //delay(2000); /* Repeat after delay */

  /* - end FFT calculations - */
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case FFT_SCALE_INDEX:
        abscissa = (i * 1.0);
        break;
      case FFT_SCALE_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case FFT_SCALE_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    Serial.print(abscissa, 6);
    if (scaleType == FFT_SCALE_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
