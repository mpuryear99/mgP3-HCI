#include <stdint.h>

const uint8_t servoLPin = D2;
const uint8_t servoRPin = D3;


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

  uint m_currPos = -1;

 public:
  ESP32Servo(uint8_t pin, uint8_t channel, uint32_t dutyCycleOffset=0)
  : m_pin(pin), m_ch(channel)
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
    uint32_t dutyCycle = m_dclo + pos/180.0*m_dcrange;
    ledcWrite(m_ch, dutyCycle);
  }

  void enable() {
    ledcAttachPin(m_pin, m_ch);
  }

  void disable() {
    ledcDetachPin(m_pin);
  }

  void setup(int pos = 0) {
    ledcSetup(m_ch, m_freq, m_res);
    this->enable();
    this->setPosition(pos);
  }

};

ESP32Servo servoL(servoLPin, 0, -50);
ESP32Servo servoR(servoRPin, 1, -100);

void setup() {
  Serial.begin(115200);
  servoL.setup(0);
  servoR.setup(180);
}


int pos = 0;
int inc = 15;
void loop() {
  // put your main code here, to run repeatedly:
  pos += inc;
  if (!(pos % 180)) inc *= -1;
  Serial.println(pos);
  servoL.setPosition(pos);
  servoR.setPosition(180-pos);
  delay(1000);
  // if (pos == 0)
  //   pos = 180;
  // else pos = 0;
}
