/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Braccio++.h>

#include "AppState.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

CalibrateAndTestApp app;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  if (Braccio.begin(custom_main_menu)) {
    app.enableButtons();
    /* Allow greater angular velocity than the default one. */
    //Braccio.setAngularVelocity(45.0f);
    Braccio.speed(speed_grade_t::SLOW);
  }
}

void loop()
{ 
  /* Only execute every 1500 ms. */
  static auto prev = millis();
  auto const now = millis();

  if ((now - prev) > 1500)
  {
    prev = now;
    app.update(EventSource::TimerTick);
  }
}