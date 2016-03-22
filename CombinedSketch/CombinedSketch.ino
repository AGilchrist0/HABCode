#include <Wire.h>
#include <DHT.h>
#include <SPI.h>
#include <SparkFun_MS5803_I2C.h>
#include <SparkFunLSM9DS1.h>

#define DHTPIN 2 
#define DHTTYPE DHT22 

MS5803 PressureTemp(ADDRESS_HIGH);
DHT dht(DHTPIN, DHTTYPE);
LSM9DS1 imu;

//Hardware pin definitions
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board

///////////////////////
// LSM9DS1 I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 2595 // 5400 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// yours here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 43 // Declination (degrees) in Boulder, CO.

//Create variables to store results
String timer;
double temperature, pressure_abs, pressure_relative, altitude_delta, uv_intensity, humidity, heatIndex, pressure_baseline;
int uvLevel, refLevel;
double mx, my, mz, gx, gy, gz, ax, ay, az;

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
  dht.begin();
  imu.begin();
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
}

void loop() {
  timer = "'time':'"+String(millis())+"', "; 
  temperature = PressureTemp.getTemperature(CELSIUS, ADC_512);
  pressure_abs= PressureTemp.getPressure(ADC_4096);
  pressure_relative = sealevel(pressure_abs_value, base_altitude);
  altitude_delta = altitude(pressure_abs_value, pressure_baseline);
  humidity = dht.readHumidity();
  imu.readMag();
  mx = imu.calcMag(imu.mx);
  my = imu.calcMag(imu.my);
  mz = imu.calcMag(imu.mz);

  imu.readGyro();
  gx = imu.calcGyro(imu.gx);
  gy = imu.calcGyro(imu.gy);
  gz = imu.calcGyro(imu.gz);

  imu.readAccel();
  ax = imu.calcAccel(imu.ax);
  ay = imu.calcAccel(imu.ay);
  az = imu.calcAccel(imu.az);
  
  uvLevel = averageAnalogRead(UVOUT);
  refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  uv_intensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("{");
  Serial.print(timer);
  Serial.print("'temperature':'"+String(PressureTemp.getTemperature(CELSIUS, ADC_512))+"', ");
  Serial.print("'pressure_abs':'"+String(pressure_abs)+"', ");
  Serial.print("'pressure_relative':'"+String(pressure_relative)+"', ");
  Serial.print("'altitude_delta':'"+String(altitude_delta)+"', ");
  Serial.print("'humidity':'"+String(humidity)+"', ");
  Serial.print("'uv_intensity':'"+String(uv_intensity)+"', ");
  Serial.print("'mx':'"+String(mx)+"', ");
  Serial.print("'my':'"+String(my)+"', ");
  Serial.print("'mz':'"+String(mz)+"', ");
  Serial.print("'ax':'"+String(ax)+"', ");
  Serial.print("'ay':'"+String(ay)+"', ");
  Serial.print("'az':'"+String(az)+"', ");
  Serial.print("'gx':'"+String(gx)+"', ");
  Serial.print("'gy':'"+String(gy)+"', ");
  Serial.print("'gz':'"+String(gz)+"', ");
  Serial.print("'heading':'"+String(heading)+"', ");
  Serial.print("'pitch':'"+String(pitch)+"', ");
  Serial.print("'roll':'"+String(roll)+"'");
  Serial.print("}");
  Serial.println();
  delay(2000);
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
