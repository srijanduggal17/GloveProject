#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("Euler X: ");
  Serial.print(euler.x());
  Serial.print("Euler Y: ");
  Serial.print(euler.y());
  Serial.print("Euler Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Display the floating point data */
  Serial.print("Gyro X: ");
  Serial.print(gyro.x());
  Serial.print("Gyro Y: ");
  Serial.print(gyro.y());
  Serial.print("Gyro Z: ");
  Serial.print(gyro.z());
  Serial.print("\t\t");

  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* Display the floating point data */
  Serial.print("Linaccel X: ");
  Serial.print(linaccel.x());
  Serial.print("Linaccel Y: ");
  Serial.print(linaccel.y());
  Serial.print("Linaccel Z: ");
  Serial.print(linaccel.z());
  Serial.print("\t\t");

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /* Display the floating point data */
  Serial.print("Accel X: ");
  Serial.print(accel.x());
  Serial.print("Accel Y: ");
  Serial.print(accel.y());
  Serial.print("Accel Z: ");
  Serial.print(accel.z());
  Serial.print("\t\t");


  imu::Vector<3> magne = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  /* Display the floating point data */
  Serial.print("Magnetometer X: ");
  Serial.print(magne.x());
  Serial.print("Magnetometer Y: ");
  Serial.print(magne.y());
  Serial.print("Magnetometer Z: ");
  Serial.print(magne.z());
  Serial.print("\t\t");

  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  /* Display the floating point data */
  Serial.print("Grav X: ");
  Serial.print(grav.x());
  Serial.print("Grav Y: ");
  Serial.print(grav.y());
  Serial.print("Grav Z: ");
  Serial.print(grav.z());
  Serial.print("\t\t");

  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("Quat W: ");
  Serial.print(quat.w(), 4);
  Serial.print("Quat X: ");
  Serial.print(quat.y(), 4);
  Serial.print("Quat Y: ");
  Serial.print(quat.x(), 4);
  Serial.print("Quat Z: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  
  /* Display calibration status for each sensor. */
  uint8_t system, gyros, accels, mags = 0;
  bno.getCalibration(&system, &gyros, &accels, &mags);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyros, DEC);
  Serial.print(" Accel=");
  Serial.print(accels, DEC);
  Serial.print(" Mag=");
  Serial.println(mags, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
