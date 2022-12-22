// Manage a toggle state timer, for example flashing of an LED or a flashing indicator on a display

#ifndef _STATE_TOGGLE_TIMER
#define _STATE_TOGGLE_TIMER

class StateToggleTimer
{
  public:
    StateToggleTimer(const unsigned long duration) 
    {
      m_bIsOn = false;
      m_bEnabled = false;
      m_duration = duration;
      m_timer = millis();
    }

    inline bool IsOn() { return m_bIsOn; }

    inline void SetDuration(const unsigned long duration) { m_duration = duration; }

    void Start()
    {
      m_bIsOn = true;
      m_bEnabled = true;
      m_timer = millis();
    }

    void Stop()
    {
      m_bIsOn = true;
      m_bEnabled = false;
    }

    void Update(const unsigned long now)
    {
      if (m_bEnabled)
      {
        if ((now - m_timer) > m_duration)
        {
          m_timer = now;
          m_bIsOn = !m_bIsOn;
        }
      }
      else
      {
        m_bIsOn = true;
      }
    }

  private:
    bool m_bIsOn;               // Flashing changes this state between on/off
    bool m_bEnabled;            // Is the flash system enabled?
    unsigned long m_duration;   // Duration of a flash
    unsigned long m_timer;      // Timer to control the flash
};

#endif