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
    _targetPos = absolute;
    computeNewSpeed();
    if(_stepInterval == 0){ // motor is stopped, thus update lastStepTime
      _lastStepTime = CUR_TIME;
    }
    // compute new n?
  }
}

void AccelStepper::move(long relative)
{
  moveTo(_currentPos + relative);
}

long AccelStepper::distanceToGo()
{
  return _targetPos - _currentPos;
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
// Sets speed to 0
void AccelStepper::setCurrentPosition(long position)
{
  _targetPos = _currentPos = position;
  _n = 0;
  _stepInterval = 0;
  _speed = 0.0;
}

void AccelStepper::computeNewSpeed()
{
  long distanceTo = distanceToGo(); // +ve is clockwise from curent location

  long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

  if (distanceTo == 0 && stepsToStop <= 1)
  {
    // We are at the target and its time to stop
    _stepInterval = 0;
    _speed = 0.0;
    _n = 0;
    return;
  }

  if (distanceTo > 0)
  {
    // We are anticlockwise from the target
    // Need to go clockwise from here, maybe decelerate now
    if (_n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
        _n = -stepsToStop; // Start deceleration
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
        _n = -_n; // Start accceleration
    }
  }
  else if (distanceTo < 0)
  {
    // We are clockwise from the target
    // Need to go anticlockwise from here, maybe decelerate
    if (_n > 0)
    {
      // Currently accelerating, need to decel now? Or maybe going the wrong way?
      if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
        _n = -stepsToStop; // Start deceleration
    }
    else if (_n < 0)
    {
      // Currently decelerating, need to accel again?
      if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
        _n = -_n; // Start accceleration
    }
  }

  // Need to accelerate or decelerate
  if (_n == 0)
  {
    // First step from stopped
    _cn = _c0;
    _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  else
  {
    // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
    _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
    _cn = max(_cn, _cmin); 
  }
  _n++;
  _stepInterval = _cn;
  _speed = 1000000.0 / _cn;
  if (_direction == DIRECTION_CCW)
    _speed = -_speed;

#if 0
  Serial.println(_speed);
  Serial.println(_acceleration);
  Serial.println(_cn);
  Serial.println(_c0);
  Serial.println(_n);
  Serial.println(_stepInterval);
  Serial.println(distanceTo);
  Serial.println(stepsToStop);
  Serial.println("-----");
#endif
}

AccelStepper::AccelStepper(uint8_t step_pin_shift, uint8_t dir_pin_shift)
{
  _currentPos = 0;
  _targetPos = 0;
  _speed = 0.0;
  _maxSpeed = 1.0;
  _acceleration = 0.0; // this is later set to default value
  _stepInterval = 0;
  _enablePin = 0xff;
  _lastStepTime = CUR_TIME;
  _step_pin = step_pin_shift;
  _dir_pin = dir_pin_shift;
  _enableInverted = false;

  // NEW
  _n = 0;
  _c0 = 0.0;
  _cn = 0.0;
  _cmin = 1.0;
  _direction = DIRECTION_CCW;

  // Some reasonable default
  setAcceleration(1);
}

void AccelStepper::setMaxSpeed(float speed)
{
  if (speed < 0.0)
    speed = -speed;
  if (_maxSpeed != speed)
  {
    _maxSpeed = speed;
    _cmin = 1000000.0 / speed;
    // Recompute _n from current speed and adjust speed if accelerating or cruising
    if (_n > 0)
    {
      _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
      computeNewSpeed();
    }
  }
}

float   AccelStepper::maxSpeed()
{
  return _maxSpeed;
}

void AccelStepper::setAcceleration(float acceleration)
{
  if (acceleration == 0.0)
    return;
  if (acceleration < 0.0)
    acceleration = -acceleration;
  if (_acceleration != acceleration)
  {
    // Recompute _n per Equation 17
    _n = _n * (_acceleration / acceleration);
    // New c0 per Equation 7, with correction per Equation 15
    _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
    _acceleration = acceleration;
    computeNewSpeed();
  }
}

float AccelStepper::getAcceleration() const 
{
  return _acceleration;
}

void AccelStepper::setSpeed(float speed)
{
  if (speed == _speed)
    return;
  speed = constrain(speed, -_maxSpeed, _maxSpeed);
  if (speed == 0.0)
    _stepInterval = 0;
  else
  {
    _stepInterval = fabs(1000000.0 / speed);
    _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
  }
  _speed = speed;
}

float AccelStepper::speed()
{
  return _speed;
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
  if (_speed != 0.0)
  {    
    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
    if (_speed > 0)
      move(stepsToStop);
    else
      move(-stepsToStop);
  }
}

bool AccelStepper::isRunning()
{
  return !(_speed == 0.0 && _targetPos == _currentPos);
}
