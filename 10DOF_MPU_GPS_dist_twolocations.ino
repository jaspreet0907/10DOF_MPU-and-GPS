// This code is integration of 10 DOF MPU Sensor (MPU 6050) and GPS Sensor.
// It calculates the distance between two coordinate position.

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>

//Defining pin and Baud Rate for GPS
static const int RXPin = 12, TXPin = 13;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// Serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);

// Defining target lat and long
const float des_lat = 12.9141;
const float des_long = 74.8560;


Adafruit_MPU6050 mpu;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}



void setup(void) {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  Serial.begin(9600);
  Serial.println("Pressure Sensor Test"); Serial.println("");
  
  // Initializes the BMP sensor 
  if(!bmp.begin())
  {
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  // Display the information of Sensor
  displaySensorDetails();

 mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void loop() {

  //GPS
  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        double latitude = gps.location.lat();
        Serial.print("Curr_lat: ");
        Serial.print(gps.location.lat(), 5);
        Serial.print('\t');

        double longitude = gps.location.lng();
        Serial.print("Curr_long: ");
        Serial.print(gps.location.lng(), 5);
        Serial.print('\t');

        double distancemeter = TinyGPSPlus::distanceBetween(latitude, longitude, des_lat, des_long) ; //Distance btween two positions.

        //Target Latitude and Longitude
        Serial.print("Des_lat: ");
        Serial.print(des_lat, 5);
        Serial.print('\t');

        Serial.print("Des_long: ");
        Serial.print(des_long, 5);
        Serial.print('\t');

        Serial.print("Dist: ");
        Serial.print(distancemeter);
        Serial.print('\t');
        
        //Speed
        Serial.print("Speed(mph): ");
        Serial.print(gps.speed.mph());
        Serial.print('\t');

      } else {
        Serial.print("- location: INVALID");
      }

      // MPU Readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Acceleration of the Body
      Serial.print("Acceleration X: ");
      Serial.print(a.acceleration.x);
      Serial.print('\t');
      Serial.print("Acceleration Y: ");
      Serial.print(a.acceleration.y);
      Serial.print('\t');
      Serial.print("Acceleration Z: ");
      Serial.print(a.acceleration.z);
      Serial.print('\t');

      // Rotation of the Body
      Serial.print("Rotation X: ");
      Serial.print(g.gyro.x);
      Serial.print('\t');
      Serial.print("Rotation Y: ");
      Serial.print(g.gyro.y);
      Serial.print('\t');
      Serial.print("Rotation Z: ");
      Serial.print(g.gyro.z);
      Serial.print('\t');

  sensors_event_t event;
  bmp.getEvent(&event);
 if (event.pressure)
  {
    // Atmospheric Pressure in hPA
    Serial.print("Pressure: ");
    Serial.print(event.pressure);
    Serial.print(" hPa");
    Serial.print('\t');

    // Altitude in meters
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude: "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure)); 
    Serial.print(" m");
    Serial.println('\t');
  }
  else
  {
    Serial.println("Sensor error");
  }
    delay(100);
    }
  }
 }
  
