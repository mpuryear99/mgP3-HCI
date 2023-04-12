#include <stdint.h>
#include "ICM_20948.h"
#include <Wire.h>

const uint8_t servoLPin = D2;
const uint8_t servoRPin = D3;
ICM_20948_I2C myICM;


void gyroToXYZ(const icm_20948_DMP_data_t &data, int32_t *x_out, int32_t *y_out, int32_t *z_out) {

  // We have asked for GRV data so we should receive Quat6
  if (!((data.header & DMP_header_bitmap_Quat9) > 0))
    return;

  //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

  // Scale to +/- 1
  double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
  double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
  double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

  // Convert the quaternions to Euler angles (roll, pitch, yaw)
  // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

  // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
  // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
  // The quaternion data is scaled by 2^30.

  double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
  double q2sqr = q2 * q2;

  // roll (x-axis rotation)
  double t0 = +2.0 * (q0 * q1 + q2 * q3);
  double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
  double roll = atan2(t0, t1) * 180.0 / PI;
  *x_out = roll;

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q0 * q2 - q3 * q1);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = asin(t2) * 180.0 / PI;
  *y_out = pitch;

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q0 * q3 + q1 * q2);
  double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
  double yaw = atan2(t3, t4) * 180.0 / PI;
  *z_out = yaw;
}


void updateAngles(icm_20948_DMP_data_t *data, int *x_out, int *y_out, int *z_out) {
  while (true) {
    myICM.readDMPdataFromFIFO(data);
    while (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail) {
      myICM.readDMPdataFromFIFO(data);
    }
    if (myICM.status == ICM_20948_Stat_Ok) {
      gyroToXYZ(*data, x_out, y_out, z_out);
      return;
    }
  }
}


class ESP32Servo {
 private:
  const uint8_t m_pin;
  const uint8_t m_ch;
  const uint8_t m_freq = 50;
  const uint8_t m_res = 16;
  // const uint m_dcRange = (1<<m_res)/20;
  uint32_t m_dclo;
  uint32_t m_dchi;
  uint32_t m_dcrange;
  bool m_invert;

  uint m_currPos = -1;

 public:
  ESP32Servo(uint8_t pin, uint8_t channel, uint32_t dutyCycleOffset=0, bool invertPos=false)
  : m_pin(pin), m_ch(channel), m_invert(invertPos)
  {
    m_dclo = 2000;   // 1640;
    m_dchi = 8000;
    m_dcrange = m_dchi - m_dclo;
    m_dclo += dutyCycleOffset;
    m_dchi += dutyCycleOffset;
  }

  void setPosition(uint pos) {
    if (pos > 180)
      pos = 180;
    if (pos == m_currPos)
      return;
    m_currPos = pos;
    // uint32_t dutyCycle = m_dcRange + m_dcRange*pos/180.0;
    if (m_invert)
      pos = 180-pos;
    uint32_t dutyCycle = m_dclo + pos/180.0*m_dcrange;
    ledcWrite(m_ch, dutyCycle);
  }

  void stepPosition(int step) {
    if (!step) return;
    if ((step <= -180) || (((int)m_currPos) + step <= 0))
      setPosition(0);
    else if ((step >= 180) || (m_currPos + step >= 180))
      setPosition(180);
    else
      setPosition(m_currPos + step);
  }

  void enable() {
    ledcAttachPin(m_pin, m_ch);
  }

  void disable() {
    ledcDetachPin(m_pin);
  }

  uint getPosition() { return m_currPos; }

  void setup(int pos = 90) {
    ledcSetup(m_ch, m_freq, m_res);
    this->enable();
    this->setPosition(pos);
  }

};



ESP32Servo servoL(servoLPin, 0, -425, false);    // -550
ESP32Servo servoR(servoRPin, 1,  175, true);     //   50


void setup() {
  Serial.begin(115200);
  servoL.setup(90);
  servoR.setup(90);

  // Setup Gyro (from IMU library examples)
  Wire.begin();
  Wire.setClock(400000);
  while (1)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(Wire, 1);

    if (myICM.status == ICM_20948_Stat_Ok)
      break;

    Serial.println("Waiting on gyro...");
    delay(500);
  }

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while(1);
  }

  Serial.println("Calibrating... (20s)");
  delay(20000);
}


icm_20948_DMP_data_t gdata;

int currPos;
int32_t x,y,z;

uint8_t state = 0;
uint8_t old_state = 0;
int8_t delta;

void loop() {
  // put your main code here, to run repeatedly:

  updateAngles(&gdata, &x, &y, &z);
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.print(z);

  if (abs(y) > 85) {
    state = 0;
    Serial.println("  | Vertical");

  } else if (abs(y) > 40) {
    state = 1;
    Serial.println("  | Almost Vertical");
    if (state == old_state) {
      delta = 4;
      if (abs(y) > 70)
        delta /= 2;
      if (abs(x) < 90)
        delta *= -1;

      servoL.stepPosition(delta);
      servoR.stepPosition(delta);
    }

  } else if (abs(x) > 130) {
    state = 2;
    Serial.println("  | Backwards");
    if (state == old_state) {
      do {
        servoL.stepPosition(-4);
        servoR.stepPosition(-4);
        delay(100);
      } while (servoL.getPosition() > 0);
      do {
        servoL.stepPosition(2);
        servoR.stepPosition(2);
        delay(100);
        if (servoL.getPosition() == 180) break;
        updateAngles(&gdata, &x, &y, &z);        
      } while (abs(y) < 70);
    }

  } else if (abs(x) < 40) {
    state = 3;
    Serial.println("  | Forward");
    if (state == old_state) {
      do {
        servoL.stepPosition(4);
        servoR.stepPosition(4);
        delay(100);
      } while (servoL.getPosition() < 180);
      do {
        servoL.stepPosition(-2);
        servoR.stepPosition(-2);
        delay(100);
        if (servoL.getPosition() == 0) break;
        updateAngles(&gdata, &x, &y, &z);
      } while (abs(y) < 70);
    }  
  } else {
    state = 4;
    Serial.println("  | Fallen Sideways (Crapshoot)");
    if (state == old_state) {
      do {
        servoL.stepPosition(5);
        servoR.stepPosition(5);
        delay(100);
      } while (servoL.getPosition() < 180);
      delay(1000);
    }
  } 

  old_state = state;
  Serial.println(servoL.getPosition());
  delay(700);
}
