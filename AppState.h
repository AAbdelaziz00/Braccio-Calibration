#ifndef CALIBRATE_AND_TEST_APP_STATE_H_
#define CALIBRATE_AND_TEST_APP_STATE_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Braccio++.h>
#include <vector>
#include <array>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/ 
const float CALIBRATION_TEST_POSITIONS[2][6] = {
  {157.50, 157.50, 119.75, 77.63, 149.12,  66.04}, 
  {157.50, 157.50, 124.45, 59.80, 162.25, 102.53}
}; 


/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class EventSource
{
  Button_ZeroPosition, Button_Calibrate, Button_Calibration_Test,  
  Button_Forward_Kinematics, 
  Button_Back, Button_Next, 
  TimerTick
};

enum class StateName
{
  Idle, Zero, Calibrate, Calibration_Test, Forward_Kinematics
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void custom_main_menu();

void moveArmToPos(const float* requested_pos);

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class State
{
public:
  virtual ~State() { }
  virtual void onEnter() { Serial.println("State::onEnter"); }
  virtual void onExit() { Serial.println("State::onExit"); }
  virtual StateName name() = 0;
  State * update(EventSource const evt_src)
  {
    State * next_state = this;
    switch (evt_src)
    {
      case EventSource::Button_ZeroPosition:       next_state = handle_OnZeroPosition();       break;
      case EventSource::Button_Calibrate:          next_state = handle_OnCalibrate();          break;
      case EventSource::Button_Calibration_Test:   next_state = handle_OnCalibrationTest();    break;
      case EventSource::Button_Forward_Kinematics: next_state = handle_OnForward_Kinematics(); break;
      case EventSource::Button_Back:               next_state = handle_OnBack();      break;
      case EventSource::Button_Next:               next_state = handle_OnNext();      break;
      case EventSource::TimerTick:                 next_state = handle_OnTimerTick(); break;
    }
    return next_state;
  }

protected: 
  virtual State * handle_OnZeroPosition()        { return this; }
  virtual State * handle_OnCalibrate()           { return this; }
  virtual State * handle_OnCalibrationTest()     { return this; }
  virtual State * handle_OnForward_Kinematics()  { return this; } 
  virtual State * handle_OnBack()      { return this; }
  virtual State * handle_OnNext()      { return this; }
  virtual State * handle_OnTimerTick() { return this; }
};

class IdleState : public State
{
public:
  virtual ~IdleState() { }
  virtual StateName name() override { return StateName::Idle; }
  virtual void onEnter() override;
  virtual void onExit() override;

protected: 
  virtual State * handle_OnCalibrate          () override;
  virtual State * handle_OnZeroPosition       () override;
  virtual State * handle_OnCalibrationTest    () override;
  virtual State * handle_OnForward_Kinematics () override; 
};

class ZeroState : public State
{
public:
           ZeroState() { }
  virtual ~ZeroState() { }
  virtual StateName name() override { return StateName::Zero; }
  virtual void onEnter()   override;
  virtual void onExit()    override;

protected:
  virtual State * handle_OnTimerTick() override;
};

class CalibrateState : public State
{
public:
           CalibrateState() : _step_count{0} { }
  virtual ~CalibrateState() { }
  virtual StateName name() override { return StateName::Calibrate; }
  virtual void onEnter()   override;
  virtual void onExit()    override;

protected:
  virtual State * handle_OnBack () override;
  virtual State * handle_OnNext () override;

private:
  int _step_count; 
  float _joint_positions[6];
};


class CalibrationTestState : public State
{
public:
           CalibrationTestState() : _step_count{0} { _total_steps = sizeof(CALIBRATION_TEST_POSITIONS)/sizeof(CALIBRATION_TEST_POSITIONS[0]); }
  virtual ~CalibrationTestState() { }
  virtual StateName name() override { return StateName::Calibration_Test; }
  virtual void onEnter()   override;
  virtual void onExit()    override;

protected:
  virtual State * handle_OnBack () override;
  virtual State * handle_OnNext () override;

private:
  int _step_count, _total_steps;  
};

class ForwardKinematicsState : public State
{
public:
           ForwardKinematicsState() { }
  virtual ~ForwardKinematicsState() { }
  virtual StateName name() override { return StateName::Forward_Kinematics; }
  virtual void onEnter()   override;
  virtual void onExit()    override;

protected:
  virtual State * handle_OnBack () override; 
  virtual State * handle_OnTimerTick() override;
}; 

class CalibrateAndTestApp
{
public:
  CalibrateAndTestApp()
  : _state{new IdleState()}
  , _mtx{}
  { }

  void enableButtons();

  void update(EventSource const evt_src)
  {
    mbed::ScopedLock<rtos::Mutex> lock(_mtx);
    
    if (!_state)
    {
      _state = new IdleState();
      _state->onEnter();
      return;
    }
    
    State * next_state = _state->update(evt_src);
      
    if (next_state->name() != _state->name())
    {
      _state->onExit();
      delete _state;
      _state = next_state;
      _state->onEnter();
    }    
  }

private:
  State * _state;
  rtos::Mutex _mtx;
};

#endif /* CALIBRATE_AND_TEST_APP_STATE_H_ */