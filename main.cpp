#include <Arduino.h>
#include "ICM_20948.h" 

#define SERIAL_PORT Serial
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.      

#define ACC 0
#define GYR 1
#define MAG 2

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void printPaddedInt16b(int16_t val);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
void PrintCalibratedValue(String name, float &max, float &min);
void PrintIMUData(float a, float b, float c);
void Get_max_min(float a, float &max, float &min);
void printScaledAGMT(ICM_20948_I2C *sensor, int times, int s);

void setup(){
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized){

    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok){
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else{
      initialized = true;
    }
  }
}

void loop(){
  printScaledAGMT(&myICM, 1000, ACC);
  printScaledAGMT(&myICM, 1000, GYR);
  printScaledAGMT(&myICM, 1000, MAG);
  while(1);
}

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}
void Get_max_min(float a, float &max, float &min){
	
	if(a > max){
	 max = a;
	}
	
	if(a < min){
	 min = a;
	}
}
void PrintCalibratedValue(String name, float &max, float &min){
  SERIAL_PORT.print(name); SERIAL_PORT.print(": "); SERIAL_PORT.println((max+min)/2);
}

void printScaledAGMT(ICM_20948_I2C *sensor, int times, int s){  
  float x = 0, y=0, z=0; //axis variables
  float mx = 0, my = 0, mz = 0; //min axis variables
  float Mx = 0, My = 0, Mz = 0; //max axis variables

  for(int k=0; k<times; k++){
    if (myICM.dataReady()){
      myICM.getAGMT(); //update IMU data

      if(s == ACC){ //accelerometer data
        if(k == 0) SERIAL_PORT.println("Acc (mg): ");
        x= sensor->accX();
        y = sensor->accY();
        z = sensor->accZ();
      }

      if(s == GYR){ //gyroscope data
        if(k == 0) SERIAL_PORT.println("Gyr (DPS): ");
        x = sensor->gyrX();
        y = sensor->gyrY();
        z = sensor->gyrZ();
      }

      if(s == MAG){ //magnetometer data
        if(k == 0) SERIAL_PORT.println("Mag (uT): ");
        x = sensor->magX();
        y = sensor->magY();
        z = sensor->magZ();
      }
    
      PrintIMUData(x, y, z);
      if(k < times - 1) SERIAL_PORT.print(","); 

      Get_max_min(x, Mx, mx);
      Get_max_min(y, My, my);
      Get_max_min(z, Mz, mz);
      
      delay(1);
    }
  }

  if(s == ACC){ //accelerometer data
    SERIAL_PORT.println("Acc (mg): ");
    PrintCalibratedValue("ax", Mx, mx);
    PrintCalibratedValue("ay", My, my);
    PrintCalibratedValue("az", Mz, mz);
  }

  if(s == GYR){ //gyroscope data
    SERIAL_PORT.println("Gyr (DPS): ");
    PrintCalibratedValue("gx", Mx, mx);
    PrintCalibratedValue("gy", My, my);
    PrintCalibratedValue("gz", Mz, mz);
  }

  if(s == MAG){ //magnetometer data
    SERIAL_PORT.println("Mag (uT): ");
    PrintCalibratedValue("mx", Mx, mx);
    PrintCalibratedValue("my", My, my);
    PrintCalibratedValue("mz", Mz, mz);
  }
  
}

void PrintIMUData(float a, float b, float c)
{
  SERIAL_PORT.print("[ "); 
  printFormattedFloat(a, 5, 2); SERIAL_PORT.print(", "); 
  printFormattedFloat(b, 5, 2); SERIAL_PORT.print(", "); 
  printFormattedFloat(c, 5, 2); SERIAL_PORT.print("]"); 
}

/*
roll = float(items[0]);
pitch = float(items[1]);
yaw = float(items[2]);
*/
