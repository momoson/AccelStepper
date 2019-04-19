// AccelStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper.cpp,v 1.23 2016/08/09 00:39:10 mikem Exp $

#include "AccelStepper.h"

#define CUR_TIME (TCNT1L | (unsigned int)((TCNT1H << 8)));

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

// true if overflow, false if not
bool AccelStepper::generateRamp()
{
  float v = _speed_start;
  _delta_t[0] = (unsigned int)(1000000.0/v) >> 2;
  _cnt_delta_t[0] = 0;
  _ramp_steps = 1;
  unsigned int i=0;
  unsigned int delta_t_temp;
  bool overflow = false;
  while(true)
  {
    v += _delta_t[i] * _acceleration;
    delta_t_temp = ((unsigned int)(1000000.0/v)) >> 2;
    if(delta_t_temp <= _minStepInterval) break;// we are done with the ramp, congratulations guys
    if(_delta_t[i]-delta_t_temp > min_delta_delta_t || _cnt_delta_t[i] == 255)
    { // whoa, some hardcore changes here or no more place for steps at this position
      ++i;
      if(i>=delta_t_N){
        Serial.println("OVERFLOW RAMP BUFFER");
        overflow = true;
        --i; // imax generation should still be working
        break;
      }
      _cnt_delta_t[i] = 0; //one step at this delta_t so far
      ++_ramp_steps;
      _delta_t[i] = delta_t_temp; // store the delta_t
    } else { // delta_t did almost not change
      ++_cnt_delta_t[i]; // just increase the steps to do at this delta_t
      ++_ramp_steps;
    }
  }
  _imax = i; // this is guaranteed larger than _minStepInterval
  return overflow;
}

void AccelStepper::moveTo(long absolute)
{
  move(absolute - _currentPos);
}

void AccelStepper::move(long relative)
{
  if(relative == 0) return;
  while(_state != 0); // block until movement done
  _lastStepTime = CUR_TIME; // update initial step time
  _direction = (relative > 0? DIRECTION_CW : DIRECTION_CCW);

  if(relative < 0) relative *=-1; // always positive

  //prepare stepInterval
  _stepInterval = _delta_t[0];
  _index = 0;
  _cnt_delta_t_current = _cnt_delta_t[0];

  //Serial.print("NEW MOVE: ");
  //Serial.print(_ramp_steps);
  //Serial.print(" ");
  //Serial.println(relative);

  // calculate 
  if(relative == 1){
     // due to _index == 0 and
     _cnt_delta_t_current = 0; // decrease will stop after one step
    _state = 1;
  } else {
    if(2*_ramp_steps <= relative){ // can reach max speed
      _max_i_inc_dec = _imax;
      _max_cnt_inc_dec = _cnt_delta_t[_imax];
      _cnt_const = relative - 2*_ramp_steps;
      _const_delta_t = _minStepInterval;
    } else { // cannot reach maximum speed
      if(relative % 2 ==1){ // odd, need one const step
        _cnt_const = 1;
      } else { // even, no const step
        _cnt_const = 0;
      }
      relative /= 2; //steps in increasing state
      unsigned int steps = 0;
      byte i;
      for(i = 0; i <= _imax && steps<relative; ++i){
        steps += _cnt_delta_t[i] + 1;
      }
      --i;
      _max_i_inc_dec = i;
      _max_cnt_inc_dec = _cnt_delta_t[i] - (steps - relative);
      _const_delta_t = _delta_t[i]; // is not used if even
    }
    _state = 3; // start the movement
  }
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

  _index = 0;
  _speed_start = 100;
  _state = 0;
}

void AccelStepper::setMaxSpeedAcc(float speed, float acc)
{
  if(speed > 0.0){
    _minStepInterval = (unsigned int)(1000000.0/speed) >> 2; // in 4 us granularity
  }
  if(acc > 0.0){
    _acceleration = acc*4.0/1000000.0; // internally acceleration is needed in #steps/s/4us
  }
  if(generateRamp()){ // if overflow
    _minStepInterval = _delta_t[_imax];
    //Serial.print("Over flow ramp buffer. Clipped max speed at ");
    //Serial.println(250000.0/_minStepInterval);
  }
  _stepInterval = _delta_t[0];
  _index = 0;
  _cnt_delta_t_current = _cnt_delta_t[0];
}

bool AccelStepper::get_step(byte & step, byte & dir)
{
  unsigned int time = CUR_TIME;
  return get_step(step, dir, time);
}

bool AccelStepper::get_step(byte & step, byte & dir, unsigned int & time)
{
  if(_state == 0) return true;

  if ((time  - _lastStepTime) >= _stepInterval)
  { // a step is due
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

    switch(_state){
      case 3: // INCREASING SPEED
        if(_index < _max_i_inc_dec){ // not in last ramp block
          if(_cnt_delta_t_current == 0){ // if block is done
            ++_index;
            _stepInterval = _delta_t[_index];
            if(_index == _max_i_inc_dec){
              _cnt_delta_t_current = _max_cnt_inc_dec;
            } else {
              _cnt_delta_t_current = _cnt_delta_t[_index];
            }
          } else { // block is not done
            --_cnt_delta_t_current;
          }
        } else { // in last block
          if(_cnt_delta_t_current == 0){ // block is done // now transition into other state
            if(_cnt_const!= 0){ // state 2 is used
              _state = 2;
              _stepInterval = _const_delta_t;
            } else { // directly start decreasing
              _state = 1;
              //_index is correct
              //_stepInterval is correct
            }
            _cnt_delta_t_current = _max_cnt_inc_dec; // prepare already state 1
          } else { // block is not done
            --_cnt_delta_t_current;
          }
        }
        break;
      case 2: // const speed
        --_cnt_const;
        if(_cnt_const <= 0){ // const movement done, transition to state 1
          _state = 1;
          _stepInterval = _delta_t[_index];
          //_index from INC still correct. _cnt_delta_t_current was already prepared
        }
        break;
      case 1: // DECREASING
        if(_index > 0){ // not last block
          if(_cnt_delta_t_current == 0){ // if block is done
            --_index;
            _stepInterval = _delta_t[_index];
            _cnt_delta_t_current = _cnt_delta_t[_index];
          } else { // block is not done
            --_cnt_delta_t_current;
          }
        } else { // last block
          if(_cnt_delta_t_current != 0){ // not done yet
            --_cnt_delta_t_current;
          } else { // done completely with movement
            _state = 0;
          }
        }
        break;
      default:
        //Serial.println("We are in an unknown state");
        _state = 0;
    }
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
  switch(_state){
    case 0: case 1: // already stopped or stopping
      return;
      break;
    case 3:
      // decreasing should start when acceleration is right now
      _state = 1;
    case 2:
      // decreasing state is already prepared, only current timeInterval should be set
      noInterrupts();
      _stepInterval = _delta_t[_index];
      _state = 1;
      interrupts();
      break;
  }
}

void AccelStepper::hard_stop()
{
  _state = 0; // no steps anymore!
}

bool AccelStepper::isRunning()
{
  return !(_targetPos == _currentPos);
}
