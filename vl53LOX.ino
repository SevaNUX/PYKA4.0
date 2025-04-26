
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();



float maxdist = 250;            //I've set the maximum distance around the sensor to only 400mm. Change to any other value.



void setup() {

  Serial.begin(9600);           //Start serial port
  sensor.begin();

}



void loop() {
 
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4)
  {
    int r = measure.RangeMilliMeter;
    if (r > maxdist) r = maxdist;
    Serial.print("Dist (mm) -");          //Print the values to serial port
    Serial.println(r);
    delay(1000);


   // angle = angle + angle_step;   //Increase angle value by the angle/loop value set above (in this case 2.4ï¿½ each loop)
  }
  else
  {
    //phase failures have incorrect data
    Serial.println(" out of range ");
    delay(1000);
  }
}

//This is the magnet detection interruption routine
//----------------------------------------------

/*



#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY



float maxdist = 400;            //I've set the maximum distance around the sensor to only 400mm. Change to any other value.


void setup()
{
  Serial.begin(115200);
  Wire.begin();

  
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

  //See interrupt vector below the void loop

}

void loop()
{ 

  int r = sensor.readRangeSingleMillimeters();    //Get distance from sensor
  if (r > maxdist)                                //Limit the dsitance to maximum set distance above
  {
    r = maxdist;
  }

  Serial.print(r);
  Serial.println(" ");

  
}*/