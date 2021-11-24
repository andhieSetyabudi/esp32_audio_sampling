#ifndef flag_ship_h_
#define flag_ship_h_

enum {WAITING = 0, CLICKED, DOUBLE_CLICKED, MULTI_CLICKED, HELD = 100, RELEASED};

#define SECONDS 1000000
#define MILLISECONDS 1000
#define MICROSECONDS 1

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
//just in case someone is still using old versions
#include "WProgram.h"
#endif

class flag_ship
{
  private:
    uint8_t 	State, lastState;
	uint8_t 	lastClickedState;
    uint8_t     output, lastOut;

    unsigned long holdTime, DBInterval, RO_Time;
    unsigned long time, duration, HeldTime;
    unsigned long lastDebounceTime;
	
    void 	(*F_Clicked)();
    void 	(*F_DClicked)();
    void 	(*F_Mult)();
    void 	(*F_Hold)();

  public:
  flag_ship()
	{
	  output = 0; 
	  time = 0; 
	  SetHoldTime(); 
	  SetDebounceTime();
	}

  void SetStateAndTime(uint8_t S = HIGH, unsigned long Time = 1000) // 500 gives you enough time to be clicked about 3 times.
  {
    State = S;                            // Set the preferred state of the clicked. 
    lastClickedState = lastState = !State; //lastState should be inverted from State
	duration = Time;
  }
	
  void SetHoldTime(unsigned long Time = 1000)
	{
	  holdTime = Time; // Set the hold time in seconds
	}
	
    void SetDebounceTime(unsigned long Time = 10)
    {
      DBInterval = Time;
    }

    void ISR_clicked(void (*P)() )
    {
      F_Clicked = P;
    }
	
	void ISR_doubleClicked(void (*DP)() )
    {
      F_DClicked = DP;
    }
	
	void ISR_multiClicked(void (*MUL)() )
    {
      F_Mult = MUL;
    }
	
    void ISR_held(void (*HOLD)() )
    {
      F_Hold = HOLD;
    }
	
	uint8_t updateState(uint8_t state)
	{
	    
        if( millis() - this->time > this->duration )
        {
            if (state != this->lastClickedState)
            {
                this->lastDebounceTime = millis();
                this->lastClickedState = state;
                this->time = millis();
                this->output = 0;
            }
        }
        else
        {
            // Check for Rollover
            unsigned long RO_Time = millis(); // current time into RollOver variable
            if (RO_Time < this->time) // is the RollOver variable smaller than ontime?
                this->time = RO_Time; // if yes,  reset ontime to zero
            // button = digitalRead(ButtonPin);     // read the button
            if (state != this->lastState) // see if the button is not held down.
            {
                if (state == !this->State) // button was released
                {
                    if ((millis() - this->lastDebounceTime) >= this->DBInterval) // button debounce, very important
                    {
                        this->output++;                    // increment a counter, but only when the button is pressed then released
                        this->lastDebounceTime = millis(); // update the debounce time
                    }
                }
                this->lastState = state; // update the buttons last state with it's new state
            }
            __asm__("nop\n\t");
        }
        
        if(state == this->State && state == this->lastClickedState)
            if( (this->HeldTime = (millis() - this->time)) > this->holdTime )
            this->output = HELD; 
        
        switch (this->output)
        {
            case WAITING:
            break;
            
            case CLICKED:
            if (*F_Clicked) 
                F_Clicked();
            break;
            
            case DOUBLE_CLICKED:
            if (*F_DClicked) 
                F_DClicked();
            break;
            
            case MULTI_CLICKED:
            if (*F_Mult)
                F_Mult();
            break;
            
            case HELD:
            if (*F_Hold) 
                F_Hold();
            break;
        }
        return this->output; // return the output count
    }
	// This returns the elapsed held time
    float GetHeldTime(float divisor = SECONDS)
    {
      if (divisor > 0)
        return this->HeldTime / divisor;
      else
        return -1;
    }
};
#endif