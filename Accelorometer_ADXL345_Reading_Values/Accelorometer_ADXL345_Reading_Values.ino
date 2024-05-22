//include libraries

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

unsigned long currentSeconds = millis()/1000;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Initialising Global Variables
int loopCounter = 0;
float x_acceleration_LSB, y_acceleration_LSB, z_acceleration_LSB;
float x_acceleration_G, y_acceleration_G, z_acceleration_G;
float g_value_total;
int sensorRange;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Accelerometer Test");
  Serial.println(" ");

  /* Initialize the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1)
      ;
  }
}

void loop(void) {
  if (loopCounter < 30) {

    accel.setRange(ADXL345_RANGE_8_G);
    // Possible ranges: 2G, 4G, 8G, 16G
  
    /* Get the sensor resolution */
    
    switch(accel.getRange()){
      case 0:
        sensorRange = 2;
        break;
      
      case 1:
        sensorRange = 4;
        break;
      
      case 2:
        sensorRange = 8;
        break;
      
      case 3:
        sensorRange = 16;
        break;
    }

    sensor_t accelerometer_sensor;
    accel.getSensor(&accelerometer_sensor);
    float resolution = accelerometer_sensor.resolution;

    /* Calculate the sensor sensitivity (resolution per G) */
    float sensitivity = resolution / sensorRange;
    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);

    // Calculating values...
    x_acceleration_LSB = event.acceleration.x;
    y_acceleration_LSB = event.acceleration.y;
    z_acceleration_LSB = event.acceleration.z;

    x_acceleration_G = ((x_acceleration_LSB * sensitivity) / resolution);
    y_acceleration_G = ((y_acceleration_LSB * sensitivity) / resolution);
    z_acceleration_G = ((z_acceleration_LSB * sensitivity) / resolution);

    // Aggregate Value...
    g_value_total = sqrt(pow(x_acceleration_G, 2) + pow(y_acceleration_G, 2) + pow(z_acceleration_G, 2));

    Serial.println("======================================");
    Serial.print(loopCounter+1); Serial.println(" Reading: ");
    Serial.println("======================================");
   
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: ");
    Serial.print(x_acceleration_LSB);
    Serial.println(" m/s^2 ");
    Serial.println(" ");
    Serial.print("Y: ");
    Serial.print(y_acceleration_LSB);
    Serial.println(" m/s^2 ");
    Serial.println(" ");
    Serial.print("Z: ");
    Serial.print(z_acceleration_LSB);
    Serial.println(" m/s^2 ");
    Serial.println(" ");

    Serial.print("X: ");
    Serial.print(x_acceleration_G);
    Serial.println(" G ");
    Serial.println(" ");
    Serial.print("Y: ");
    Serial.println(y_acceleration_G);

    Serial.print("\nZ: ");
    Serial.print(z_acceleration_G);
    Serial.println(" G ");
    Serial.println(" ");

    Serial.print("The enum range of sensor: "); Serial.println(accel.getRange());
    Serial.print("The Range of sensor: "); Serial.println(sensorRange);
    Serial.print(" ");

    Serial.print("Sensor Resolution: ");
    Serial.print(resolution, 4);
    Serial.println(" LSB");
    Serial.print("Sensor Sensitivity: ");
    Serial.print(sensitivity, 4);
    Serial.println(" LSB/g");
    Serial.println(" ");

    Serial.print(" Total value in G: ");
    Serial.print(g_value_total);
    Serial.println(" G ");
    Serial.println(" ");
    Serial.print(" Time: "); Serial.print(currentSeconds); Serial.print(" seconds"); Serial.println();
    Serial.println("--------------------------------------");

    /* Delay before the next reading */
    delay(500);
    loopCounter++;
  } else {
    while (1) {
    }
  }
}

