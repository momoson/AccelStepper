// AccelStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper.cpp,v 1.23 2016/08/09 00:39:10 mikem Exp $

#include "AccelStepper.h"

#define CUR_TIME ((unsigned int)((TCNT1H << 8) | TCNT1L));

#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
  int i;

  for (i = 0; i < l; i++)
  {
    Serial.print(p[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}
#endif

void AccelStepper::moveTo(long absolute)
{
  if (_targetPos != absolute)
  {
    if(distanceToGo() == 0){ // motor is stopped, thus update lastStepTime
      _lastStepTime = CUR_TIME;
    }
    _targetPos = absolute;
    long distance_to_go = distanceToGo();
    if(distance_to_go > 0){
      _direction = DIRECTION_CW;
    } else _direction = DIRECTION_CCW;
  }
}

void AccelStepper::move(long relative)
{
  moveTo(_currentPos + relative);
}


long AccelStepper::targetPosition()
{
  return _targetPos;
}

long AccelStepper::currentPosition()
{
  return _currentPos;
}

// Useful during initialisations or after initial positioning
void AccelStepper::setCurrentPosition(long position)
{
  _targetPos = _currentPos = position;
}

AccelStepper::AccelStepper(uint8_t step_pin_shift, uint8_t dir_pin_shift)
{
  _currentPos = 0;
  _targetPos = 0;
  _stepInterval = 0;
  _enablePin = 0xff;
  _lastStepTime = CUR_TIME;
  _step_pin = step_pin_shift;
  _dir_pin = dir_pin_shift;
  _enableInverted = false;
  _direction = DIRECTION_CCW;
}

void AccelStepper::setMaxSpeed(float speed)
{
  if (speed < 0.0)
    speed = -speed;
  _stepInterval = 1000000.0/speed;
}

bool AccelStepper::get_step(byte & step, byte & dir)
{
  if(distanceToGo() == 0) return true;

  unsigned int time = CUR_TIME;
  if (((time  - _lastStepTime) << 2) >= _stepInterval)
  {
    _lastStepTime = time;
    if (_direction == DIRECTION_CW)
    {
      // Clockwise
      _currentPos += 1;
      dir |= HIGH << _dir_pin;
    }
    else
    {
      // Anticlockwise  
      _currentPos -= 1;
    }
    step |= HIGH << _step_pin;
  }
  return false;
}

// Prevents power consumption on the outputs
void    AccelStepper::disableOutputs()
{   
  if (_enablePin != 0xff)
  {
    digitalWrite(_enablePin, LOW ^ _enableInverted);
  }
}

void    AccelStepper::enableOutputs()
{
  if (_enablePin != 0xff)
  {
    digitalWrite(_enablePin, HIGH ^ _enableInverted);
  }
}

void AccelStepper::setEnablePin(uint8_t enablePin, bool enableInvert)
{
  _enablePin = enablePin;
  _enableInverted = enableInvert;
  if (_enablePin != 0xff)
  {
    pinMode(_enablePin, OUTPUT);
  }
}

void AccelStepper::stop()
{
  long stepsToStop = 5;
  if (_direction == DIRECTION_CW)
    move(stepsToStop);
  else
    move(-stepsToStop);
  }
}

bool AccelStepper::isRunning()
{
  return !(_speed == 0.0 && _targetPos == _currentPos);
}
