#ifndef _BUZZER
#define _BUZZER

class Buzzer
{
public:
  Buzzer()
  {
    m_pin = 0;
    m_timer = 0;
    m_duration = 0;
    m_bIsBuzzing = false;
  }

  void Initialize(const int pin)
  {
    m_pin = pin;
    pinMode(m_pin, OUTPUT);
    digitalWrite(m_pin, LOW);
  }

  void Buzz(const unsigned long duration)
  {
    m_bIsBuzzing = true;
    m_timer = millis();
    m_duration = duration;
    digitalWrite(m_pin, HIGH);
    DebugPrintln("Buzzing...");
  }

  void Update(const unsigned long now)
  {
    if (m_bIsBuzzing && ((now - m_timer) > m_duration))
    {
      digitalWrite(m_pin, LOW);
      m_bIsBuzzing = false;
    }
  }

private:
  int m_pin;
  bool m_bIsBuzzing;
  unsigned long m_timer;
  unsigned long m_duration;

};

#endif