
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

MS5803 PressureTemp(ADDRESS_HIGH);

//Hardware pin definitions
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board

//Create variables to store results
String timer, temperature, pressure_abs, pressure_relative, altitude_delta, uv_reading;
double pressure_baseline;

// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)
double pressure_abs_value;

void setup() {
  Serial.begin(9600);
   //Retrieve calibration constants for conversion math.
  PressureTemp.reset();
  PressureTemp.begin();
  pressure_baseline = PressureTemp.getPressure(ADC_4096);
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
}

void loop() {
  /*Serial.print("{");
  timer = "'time':'"+String(millis())+"', ";
  Serial.print(timer);
  temperature = "'temperature':'"+String(PressureTemp.getTemperature(CELSIUS, ADC_512))+"', ";
  Serial.print(temperature);
  pressure_abs_value = PressureTemp.getPressure(ADC_4096);
  pressure_abs = "'pressure_abs':'"+String(pressure_abs_value)+"', ";
  Serial.print(pressure_abs);
  pressure_relative = "'pressure_relative':'"+String(sealevel(pressure_abs_value, base_altitude))+"', ";
  Serial.print(pressure_relative);
  altitude_delta = "'altitude_delta':'"+String(altitude(pressure_abs_value, pressure_baseline))+"', ";
  Serial.print(altitude_delta);
  Serial.print("}");
  Serial.println();
  delay(1000);*/
  
  Serial.print("{");
  timer = "'time':'"+String(millis())+"', ";
  Serial.print(timer);
  temperature = "'temperature':'"+String(PressureTemp.getTemperature(CELSIUS, ADC_512))+"', ";
  Serial.print(temperature);
  pressure_abs_value = PressureTemp.getPressure(ADC_4096);
  pressure_abs = "'pressure_abs':'"+String(pressure_abs_value)+"', ";
  Serial.print(pressure_abs);
  pressure_relative = "'pressure_relative':'"+String(sealevel(pressure_abs_value, base_altitude))+"', ";
  Serial.print(pressure_relative);
  altitude_delta = "'altitude_delta':'"+String(altitude(pressure_abs_value, pressure_baseline))+"', ";
  Serial.print(altitude_delta);

  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

  uv_reading = "'uv_intensity':'"+String(uvIntensity)+"'";

  Serial.print(uv_reading);
  
  Serial.print("}");
  Serial.println();
  delay(1000);

}

//MS5803
// Thanks to Mike Grusin for letting me borrow the functions below from 
// the BMP180 example code. 

 double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));
}

//ML8511
//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
  
