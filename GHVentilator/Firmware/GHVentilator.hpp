/** Gravity Hookah Ventilator @version 0.x
@link    https://github.com/KabukiStarship/SickBay.git
@file    /GHVentilaorChannel.cpp
@author  Cale McCollough <https://cale-mccollough.github.io>
@license Copyright 2020 (C) Kabuki Starship <kabukistarship.com>.
This Source Code Form is subject to the terms of the Mozilla Public License, 
v. 2.0. If a copy of the MPL was not distributed with this file, you can obtain 
one at <https://mozilla.org/MPL/2.0/>. */

#include "GHVentilator.h"
#include <mbedBug.h>

namespace SickBay {

GHVentilator::GHVentilator ():
    Status        (1),
    Ticks         (0),
    PressureMin   (0),
    PressureMax   (1) {}

void GHVentilator::Init (int TicksSecond,
                         int TicksCalibration, 
                         float HysteresisChamber,
                         float HysteresisPatient) {
  int InhaleTicksMax = TicksSecond * 6; //<
  this->Status            = -1;
  this->Ticks             = StateRising;
  this->TicksMonitor      = TicksSecond;
  this->TicksSecond       = TicksSecond;
  this->TicksInhaleMin    = TicksSecond;
  this->TicksInhaleMax    = InhaleTicksMax;
  this->TicksExhaleMin    = TicksSecond;
  this->TicksExhaleMax    = InhaleTicksMax * 3;
  this->TicksPEEP         = TicksSecond * 5;
  this->TicksCalibration  = TicksCalibration;
  this->PressureMin       = Pressure;
  this->PressureMax       = Pressure;
  this->HysteresisChamber = HysteresisChamber;
  this->HysteresisPatient = HysteresisPatient;
  int TicksInhale = 5 * TicksSecond,
      TicksExhale = 3 * TicksInhale;
  
  for (int Index = ChannelCount - 1; Index > 0; --Index) {
    float Delta = float (Channels[Index].Pressure) * HysteresisPatient;
    Channels[Index].Init (this, TicksInhale, TicksExhale, 
                          static_cast<uint32_t>(Delta));
  }
  DPrintf("\nInitalized system with %d Ticks per Second.", TicksSecond);
}

void GHVentilator::TicksPEEPSet (int NewTicksPEEP) {
  int TicksPEEPMin = TicksSecond >> 5; //< This divides by 2^5.
  if (NewTicksPEEP < TicksPEEPMin || NewTicksPEEP > TicksSecond) {
    DPrintf("\n  > Error PEEP must be betwen 1/32 of a second to 1 second. <");
    return;
  }
  TicksPEEP = NewTicksPEEP;
}

void GHVentilator::StateCalibrateEnter () {
  TurnOff ();
  BlowerTurnOn ();
  Status = StateCalibrate; //< Make sure to set this last.
}
  
void GHVentilator::StateCalibrateExit () {
  PressureMax = Pressure;
  uint32_t HysteresisChamber = this->HysteresisChamber,
        PressureMid = (PressureMax - PressureMin) / 2;
  PressureMin = PressureMid - HysteresisChamber;
  PressureMax = PressureMid + HysteresisChamber;
  DPrintf ("\n? Done calibrating with PressureMin:%u and PressureMax%u. <",
           PressureMin, PressureMax);
}

void GHVentilator::StateSinkingEnter () {
  Ticks = StateSinking;
  BlowerTurnOff ();
}

void GHVentilator::StateRisingEnter () {
  Ticks = StateRising;
  BlowerTurnOn ();
}

void GHVentilator::TurnOff () {
  BlowerTurnOff();
  Ticks = 0;
  Status = 0;
  Channel1ValveSet (LLLow);
  #if GHVentilatorChannelCount >= 2
  Channel2ValveSet (LLLow);
  #endif
  #if GHVentilatorChannelCount >= 3
  Channel3ValveSet (LLLow);
  #endif
  #if GHVentilatorChannelCount >= 4
  Channel4ValveSet (LLLow);
  #endif
}

void GHVentilator::ChannelValveSet (GHVentilatorChannel* Channel, int Value) {
  if      (Channel == &Channels[0]) Channel1ValveSet(Value);
  else if (Channel == &Channels[1]) Channel2ValveSet(Value);
  else if (Channel == &Channels[2]) Channel3ValveSet(Value);
  else if (Channel == &Channels[3]) Channel4ValveSet(Value);
}

int GHVentilator::TicksInhaleExhaleSet (int Index, 
                                        int TicksInhale,
                                        int TicksExhale) {
  if (Index < 0 || Index >= ChannelCount) return -1;
  if (TicksInhale <= TicksInhaleMax) return -2;
  if (TicksInhale >= TicksInhaleMin) return -3;
  if (Index >= ChannelCount) return 1;
  if (TicksExhale <= TicksExhaleMax) return 2;
  if (TicksExhale >= TicksExhaleMin) return 3;
  Channels[Index].TicksInhaleExhaleSet(TicksInhale, TicksExhale);
  return 0;
}
  
void GHVentilator::Update() {
  int Tick = Ticks;
  if (Tick == 0) {
    int Status = this->Status;
    if (Status == 0) return; // We're in the Off State so do nothing.
    // We're in the Calibrating State
    if (++Status > TicksCalibration) {
      StateCalibrateExit ();
      return;
    }
    Status = Status;
    int Pressure = this->Pressure;
    if (Pressure >= PressureMax) {
      this->PressureMax = Pressure;
    }
  }
  else if (Tick <= StateSinking) {
    if (IsUnderPressure ()) StateSinkingEnter ();
    else Ticks = Ticks - 1;
  }
  else if (Tick >= StateRising) {
    if (IsOverPressure ()) StateSinkingEnter ();
    else Ticks = Ticks + 1;
  }
  // Check the Channels for state changes.
  for (int Index = ChannelCount - 1; Index >= 0; --Index) {
    GHVentilatorChannel* Channel = &Channels[Index];
    int Tick = Channel->Ticks;
    if (Tick <= GHVentilatorChannel::StateExhaling) {
      if (Tick < Channel->TicksExhale) Channel->Inhale (); 
      else Channel->Ticks = Ticks - 1;
    }
    else if (Tick >= GHVentilatorChannel::StateInhaling) {
      int TicksExhale = Channel->TicksExhale;
      if (Tick > TicksExhale) {
        if (Tick > TicksExhale - TicksPEEP) {
          if (Channel->Pressure >= Channel->PressureReference) 
            Channel->Inhale ();
        }
      } 
      else Channels->Ticks = Ticks + 1;
    }
    else {
      return;
    }
    Ticks = Tick;
  }
}
}   //< namespace SickBay
#include "GHVentilatorChannel.hpp"