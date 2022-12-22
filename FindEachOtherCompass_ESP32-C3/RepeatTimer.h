// A simple timer that automatically starts again when time runs out

#ifndef _REPEAT_TIMER
#define _REPEAT_TIMER

class RepeatTimer
{
  public:
    RepeatTimer(const unsigned long duration) 
    {
      m_duration = duration;
      m_timer = millis();
    }

    bool RanOut(const unsigned long now)
    {
      if ((now - m_timer) > m_duration)
      {
        m_timer = now;
        return true;
      }

      return false;
    }

  private:
    unsigned long m_duration;
    unsigned long m_timer;
};

#endif