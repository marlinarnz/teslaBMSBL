#include "Controller.hpp"

/////////////////////////////////////////////////
/// \brief Orchestrates the activities within the BMS via a state machine.
/////////////////////////////////////////////////
void Controller::doController() {
  static int ticks = 0;
  static int standbyTicks = 1; //1 because ticks slow down
  const int stateticks = 4;

  // 12V battery monitoring
  bat12vVoltage = (float)analogRead(INA_12V_BAT) / BAT12V_SCALING_DIVISOR ;
  if (dc2dcON_H == 0 && bat12vVoltage < DC2DC_ON_V_SETPOINT) {
    dc2dcON_H = 1;
  } else if ( dc2dcON_H == 1 && bat12vVoltage >= DC2DC_OFF_V_SETPOINT) {
    dc2dcON_H = 0;
  }

  // Battery pack heating loop
  if (!heatingON_H && bms.getAvgTemperature() < HEATING_T_SETPOINT) {
    heatingON_H = 1;
  } else if (heatingON_H && bms.getAvgTemperature() >= HEATING_T_SETPOINT) {
    heatingON_H = 0;
  }

  // Update modules and faults
  if (state != INIT) syncModuleDataObjects();

  //figure out state transition
  switch (state) {
    /**************** ****************/
    case INIT:
      if (initialized) {
        ticks = 0;
        state = STANDBY;
      }
      break;
    /**************** ****************/
    case STANDBY:
#ifdef STATECYCLING
      if (ticks >= stateticks) {
        ticks = 0;
        state = PRE_CHARGE;
      }
#else
      if (digitalRead(INH_CHARGING) == HIGH
          && !chargerInhibit) { // charging is prioritised against running
        ticks = 0;
        state = PRE_CHARGE;
      } else if (digitalRead(INH_RUN) == HIGH
                 && !dischargeInhibit) {
        ticks = 0;
        state = RUN;
      }
#endif
      break;
    /**************** ****************/
    case PRE_CHARGE:
#ifdef STATECYCLING
      if (ticks >= stateticks) {
        ticks = 0;
        state = CHARGING;
      }
#else
      if (bms.getAvgCellVolt() < MAX_CHARGE_V_SETPOINT
          && digitalRead(INH_CHARGING) == HIGH
          && !heatingON_H
          && !chargerInhibit) {
        ticks = 0;
        state = CHARGING;
      } else if (bms.getAvgCellVolt() >= MAX_CHARGE_V_SETPOINT
                 || digitalRead(INH_CHARGING) == LOW
                 || chargerInhibit) {
        ticks = 0;
        state = STANDBY;
      }
#endif
      break;
    /**************** ****************/
    case CHARGING:
#ifdef STATECYCLING
      if (ticks >= stateticks) {
        ticks = 0;
        state = POST_CHARGE;
      }
#else
      if (bms.getHighCellVolt() >= MAX_CHARGE_V_SETPOINT
          || digitalRead(INH_CHARGING) == LOW
          || chargerInhibit) {
        ticks = 0;
        state = POST_CHARGE;
      }
#endif
      break;
    /**************** ****************/
    case POST_CHARGE:
#ifdef STATECYCLING
      if (ticks >= stateticks) {
        ticks = 0;
        state = RUN;
      }
#else
      if (bms.getHighCellVolt() - bms.getLowCellVolt() < PRECISION_BALANCE_CELL_V_OFFSET) {
        ticks = 0;
        state = STANDBY;
      }
#endif
      break;
    /**************** ****************/
    case RUN:
#ifdef STATECYCLING
      if (ticks >= stateticks) {
        ticks = 0;
        state = INIT;
      }
#else
      if (digitalRead(INH_RUN ) == LOW
          || dischargeInhibit) {
        ticks = 0;
        state = STANDBY;
      }
#endif
      break;
    /**************** ****************/
    default:
      break;
  }

  //execute state
  switch (state) {

    case INIT:
      period = LOOP_PERIOD_ACTIVE_MS;
      init();
      break;

    case STANDBY:
      //prevents sleeping if the console is connected or if within 1 minute of a hard reset.
      //The teensy wont let reprogram if it slept once so this allows reprograming within 1 minute.
      if (SERIALCONSOLE || millis() < 60000) {
        period = LOOP_PERIOD_ACTIVE_MS;
        standbyTicks = 12;
      } else {
        period = LOOP_PERIOD_STANDBY_MS;
        standbyTicks = 1;
      }
      standby();
      break;

    case PRE_CHARGE:
      period = LOOP_PERIOD_ACTIVE_MS;
      pre_charge();
      break;

    case CHARGING:
      period = LOOP_PERIOD_ACTIVE_MS;
      charging();
      break;

    case POST_CHARGE:
      period = LOOP_PERIOD_ACTIVE_MS;
      post_charge();
      break;

    case RUN:
      period = LOOP_PERIOD_ACTIVE_MS;
      run();
      break;

    default:
      period = LOOP_PERIOD_ACTIVE_MS;
      break;
  }
  ticks++;
}

/////////////////////////////////////////////////
/// \brief When instantiated, the controller is in the init state ensuring that all the signal pins are set properly.
/////////////////////////////////////////////////
Controller::Controller() {
  state = INIT;
  initialized = false;
}

/////////////////////////////////////////////////
/// \brief gather all the data from the boards and check for any faults.
/////////////////////////////////////////////////
void Controller::syncModuleDataObjects() {
  bms.wakeBoards();
  bms.getAllVoltTemp();

  if (bms.getLineFault()) {
    faultBMSSerialCommsDB += 1;
    if (faultBMSSerialCommsDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultBMSSerialComms) {
        LOG_ERROR("Serial communication with battery modules lost!\n");
      }
      faultBMSSerialComms = true;
      faultBMSSerialCommsDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultBMSSerialComms) LOG_INFO("Serial communication with battery modules re-established!\n");
    faultBMSSerialCommsDB = 0;
    faultBMSSerialComms = false;
  }

  if (digitalRead(INL_BAT_PACK_FAULT) == LOW) {
    faultModuleLoopDB += 1;
    if (faultModuleLoopDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultModuleLoop) {
        LOG_ERROR("One or more BMS modules have asserted the fault loop!\n");
      }
      faultModuleLoop = true;
      faultModuleLoopDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultModuleLoop) LOG_INFO("All modules have deasserted the fault loop\n");
    faultModuleLoopDB = 0;
    faultModuleLoop = false;
  }

  if (digitalRead(INL_WATER_SENS1) == LOW) {
    faultWatSen1DB += 1;
    if (faultWatSen1DB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultWatSen1) {
        LOG_ERROR("The battery water sensor 1 is reporting water!\n");
      }
      faultWatSen1 = true;
      faultWatSen1DB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultWatSen1) LOG_INFO("The battery water sensor 1 is reporting dry.\n");
    faultWatSen1DB = 0;
    faultWatSen1 = false;
  }

  if (digitalRead(INL_WATER_SENS2) == LOW) {
    faultWatSen2DB += 1;
    if (faultWatSen2DB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultWatSen2) {
        LOG_ERROR("The battery water sensor 2 is reporting water!\n");
      }
      faultWatSen2 = true;
      faultWatSen2DB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultWatSen2) LOG_INFO("The battery water sensor 2 is reporting dry.\n");
    faultWatSen2DB = 0;
    faultWatSen2 = false;
  }

  if ( bms.getHighCellVolt() > OVER_V_SETPOINT) {
    faultBMSOVDB += 1;
    if (faultBMSOVDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultBMSOV) {
        LOG_ERROR("OVER_V_SETPOINT: %.2fV, highest cell:%.2fV\n", OVER_V_SETPOINT, bms.getHighCellVolt());
      }
      faultBMSOV = true;
      faultBMSOVDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultBMSOV) LOG_INFO("All cells are back under OV threshold\n");
    faultBMSOVDB = 0;
    faultBMSOV = false;
  }

  if ( bms.getLowCellVolt() < UNDER_V_SETPOINT) {
    faultBMSUVDB += 1;
    if (faultBMSUVDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultBMSUV) {
        LOG_ERROR("UNDER_V_SETPOINT: %.2fV, lowest cell:%.2fV\n", UNDER_V_SETPOINT, bms.getLowCellVolt());
      }
      faultBMSUV = true;
      faultBMSUVDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultBMSUV) LOG_INFO("All cells are back over UV threshold\n");
    faultBMSUVDB = 0;
    faultBMSUV = false;
  }

  if ( bms.getHighTemperature() > OVER_T_SETPOINT) {
    faultBMSOTDB += 1;
    if (faultBMSOTDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultBMSOT) {
        LOG_ERROR("OVER_T_SETPOINT: %.2fV, highest module:%.2fV\n", UNDER_V_SETPOINT, bms.getHighTemperature());
      }
      faultBMSOT = true;
      faultBMSOTDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultBMSOT) LOG_INFO("All modules are back under the OT threshold\n");
    faultBMSOTDB = 0;
    faultBMSOT = false;
  }

  if ( bms.getLowTemperature() < UNDER_T_SETPOINT) {
    faultBMSUTDB += 1;
    if (faultBMSUTDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultBMSUT) {
        LOG_ERROR("UNDER_T_SETPOINT: %.2fV, lowest module:%.2fV\n", UNDER_T_SETPOINT, bms.getLowTemperature());
      }
      faultBMSUT = true;
      faultBMSUTDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultBMSUT) LOG_INFO("All modules are back over the UT threshold\n");
    faultBMSUTDB = 0;
    faultBMSUT = false;
  }

  if ( bat12vVoltage > BAT12V_OVER_V_SETPOINT) {
    fault12VBatOVDB += 1;
    if (fault12VBatOVDB >= FAULT_DEBOUNCE_COUNT) {
      if (!fault12VBatOV) {
        LOG_ERROR("12VBAT_OVER_V_SETPOINT: %.2fV, V:%.2fV\n", BAT12V_OVER_V_SETPOINT, bat12vVoltage);
      }
      fault12VBatOV = true;
      fault12VBatOVDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (fault12VBatOV) LOG_INFO("12V battery back under the OV threshold\n");
    fault12VBatOVDB = 0;
    fault12VBatOV = false;
  }

  if ( bat12vVoltage < BAT12V_UNDER_V_SETPOINT) {
    fault12VBatUVDB += 1;
    if (fault12VBatUVDB >= FAULT_DEBOUNCE_COUNT) {
      if (!fault12VBatUV) {
        LOG_ERROR("12VBAT_UNDER_V_SETPOINT: %.2fV, V:%.2fV\n", BAT12V_UNDER_V_SETPOINT, bat12vVoltage);
      }
      fault12VBatUV = true;
      fault12VBatUVDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (fault12VBatUV) LOG_INFO("12V battery back over the UV threshold\n");
    fault12VBatUVDB = 0;
    fault12VBatUV = false;
  }

  // If the valve feedback does not match the heating state
  float feedback = (float)analogRead(INA_VALVE_FEEDBACK) / VALVE_FEEDBACK_SCALING_DIVISOR;
  if ( feedback > VALVE_CLOSED_V_THRESH && heatingON_H) {
    faultHeatLoopDB += 1;
    if (faultHeatLoopDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultHeatLoop) {
        LOG_ERROR("The cooling loop is open although in heating mode (valve feedback %.2fV)\n", feedback);
      }
      faultHeatLoop = true;
      faultHeatLoopDB = FAULT_DEBOUNCE_COUNT;
    }
  } else if ( feedback < VALVE_OPEN_V_THRESH && !heatingON_H) {
    faultHeatLoopDB += 1;
    if (faultHeatLoopDB >= FAULT_DEBOUNCE_COUNT) {
      if (!faultHeatLoop) {
        LOG_ERROR("The heating loop is closed although not in heating mode (valve feedback %.2fV)\n", feedback);
      }
      faultHeatLoop = true;
      faultHeatLoopDB = FAULT_DEBOUNCE_COUNT;
    }
  } else {
    if (faultHeatLoop) LOG_INFO("The heating/cooling loop valve works again\n");
    faultHeatLoopDB = 0;
    faultHeatLoop = false;
  }

  // Cumulative fault stati
  chargerInhibit = faultModuleLoop || faultCANbus || faultBMSSerialComms || faultBMSOV || faultBMSUT || faultBMSOT || faultWatSen1 || faultWatSen2;
  chargerInhibit |= bms.getHighCellVolt() >= MAX_CHARGE_V_SETPOINT;
  dischargeInhibit = faultModuleLoop || faultCANbus || faultBMSSerialComms || faultBMSOT || faultBMSUV || faultWatSen1 || faultWatSen2;
  isFaulted = chargerInhibit || dischargeInhibit || fault12VBatOV || fault12VBatUV || faultHeatLoop;

  heatingON_H &= !sFaultHeatLoop; // switch off heating if faulted for one or more state cycles
  heatingON_H &= !faultBMSOT; // switch off heating if over-temperature detected
  dc2dcON_H &= !dischargeInhibit; // switch off DC2DC charger if discharge disabled

  if (chargerInhibit) LOG_INFO("chargerInhibit line asserted!\n");

  //update stiky faults
  sFaultModuleLoop |= faultModuleLoop;
  sFaultCANbus |= faultCANbus;
  sFaultBMSSerialComms |= faultBMSSerialComms;
  sFaultBMSOV |= faultBMSOV;
  sFaultBMSUV |= faultBMSUV;
  sFaultBMSOT |= faultBMSUV;
  sFaultBMSUT |= faultBMSUT;
  sFault12VBatOV |= fault12VBatOV;
  sFault12VBatUV |= fault12VBatUV;
  sFaultWatSen1 |= faultWatSen1;
  sFaultWatSen2 |= faultWatSen2;
  sFaultHeatLoop |= faultHeatLoop;

  //update time stamps
  long timeInSec = long(millis() / 1000);
  if (faultModuleLoop) faultModuleLoopTS = timeInSec;
  if (faultCANbus) faultCANbusTS = timeInSec;
  if (faultBMSSerialComms) faultBMSSerialCommsTS = timeInSec;
  if (faultBMSOV) faultBMSOVTS = timeInSec;
  if (faultBMSUV) faultBMSUVTS = timeInSec;
  if (faultBMSOT) faultBMSOTTS = timeInSec;
  if (faultBMSUT) faultBMSUTTS = timeInSec;
  if (fault12VBatOV) fault12VBatOVTS = timeInSec;
  if (fault12VBatUV) fault12VBatUVTS = timeInSec;
  if (faultWatSen1) faultWatSen1TS = timeInSec;
  if (faultWatSen2) faultWatSen2TS = timeInSec;
  if (faultHeatLoop) faultHeatLoopTS = timeInSec;

  stickyFaulted |= isFaulted;
  bms.clearFaults();
  //bms.sleepBoards();
}

/////////////////////////////////////////////////
/// \brief sets the controller CAN bus fault state according to reported value
/////////////////////////////////////////////////
void Controller::reportCanStatus(bool allGood) {
  if (COMMUNICATE_VIA_CAN) {
    if (!allGood) {
      if (!faultCANbus) {
        LOG_ERROR("CAN bus fault detected!\n");
        faultCANbus = true;
        faultCANbusDB = 0;
      }
    } else {
      // switch off fault status only after debounce count
      faultCANbusDB += 1;
      if (faultCANbusDB >= FAULT_DEBOUNCE_COUNT) {
        if (faultCANbus) LOG_INFO("CAN bus fault gone\n");
        faultCANbus = false;
        faultCANbusDB = FAULT_DEBOUNCE_COUNT;
      }
    }
  }
}

/////////////////////////////////////////////////
/// \brief balances the cells according to BALANCE_CELL_V_OFFSET threshold in the CONFIG.h file
/////////////////////////////////////////////////
void Controller::balanceCells() {
  if (bms.getHighCellVolt() > PRECISION_BALANCE_V_SETPOINT) {
    //LOG_CONSOLE("precision balance\n");
    bms.balanceCells(BALANCE_CELL_PERIOD_S, PRECISION_BALANCE_CELL_V_OFFSET);
  } else if (bms.getHighCellVolt() > ROUGH_BALANCE_V_SETPOINT) {
    //LOG_CONSOLE("rough balance\n");
    bms.balanceCells(BALANCE_CELL_PERIOD_S, ROUGH_BALANCE_CELL_V_OFFSET);
  }
}

/////////////////////////////////////////////////
/// \brief computes the duty cycle required for the pwm controlling the coolant pump.
///
/// Returns a float from 0.0 - 1.0 that must be adjusted to the PWM range (0-255).
///
/// @param the temparature in C
/////////////////////////////////////////////////
//pwd = a*temp + b
#define COOLING_A (1.0 - FLOOR_DUTY_COOLANT_PUMP) / (COOLING_HIGHT_SETPOINT - COOLING_LOWT_SETPOINT)
#define COOLING_B FLOOR_DUTY_COOLANT_PUMP - COOLING_A * COOLING_LOWT_SETPOINT
float Controller::getCoolingPumpDuty(float temp) {
  if (temp < COOLING_LOWT_SETPOINT) {
    return FLOOR_DUTY_COOLANT_PUMP;
  } else if (temp > COOLING_HIGHT_SETPOINT) {
    return 1.0;
  } else {
    return COOLING_A * temp + COOLING_B;
  }
}

/////////////////////////////////////////////////
/// \brief reset all boards, assign address to each board and configure their thresholds
/////////////////////////////////////////////////
void Controller::init() {
  pinMode(OUTH_12V_BAT_CHRG, OUTPUT);
  pinMode(OUTPWM_PUMP, OUTPUT); //PWM use analogWrite(OUTPWM_PUMP, 0-255);
  pinMode(INL_BAT_PACK_FAULT, INPUT_PULLUP);
  pinMode(OUTH_BAT_HEATER, OUTPUT);
  pinMode(OUTL_VALVE_OPEN, OUTPUT);
  pinMode(INA_VALVE_FEEDBACK, INPUT); // [0-1023] = analogRead(INA_VALVE_FEEDBACK)
  pinMode(INH_RUN, INPUT_PULLDOWN);
  pinMode(INH_CHARGING, INPUT_PULLDOWN);
  pinMode(INA_12V_BAT, INPUT);  // [0-1023] = analogRead(INA_12V_BAT)
  pinMode(OUTH_OBC_ON, OUTPUT);
  pinMode(OUTH_RUN, OUTPUT);
  pinMode(INL_WATER_SENS1, INPUT_PULLUP);
  pinMode(INL_WATER_SENS2, INPUT_PULLUP);

  //faults
  faultModuleLoop = false;
  faultCANbus = false;
  faultBMSSerialComms = false;
  faultBMSOV = false;
  faultBMSUV = false;
  faultBMSOT = false;
  faultBMSUT = false;
  fault12VBatOV = false;
  fault12VBatUV = false;
  faultWatSen1 = false;
  faultWatSen2 = false;
  faultHeatLoop = false;

  //sticky faults
  sFaultModuleLoop = false;
  sFaultCANbus = false;
  sFaultBMSSerialComms = false;
  sFaultBMSOV = false;
  sFaultBMSUV = false;
  sFaultBMSOT = false;
  sFaultBMSUT = false;
  sFault12VBatOV = false;
  sFault12VBatUV = false;
  sFaultWatSen1 = false;
  sFaultWatSen2 = false;
  sFaultHeatLoop = false;

  //faults debounce counters
  faultModuleLoopDB = 0;
  faultCANbusDB = 0;
  faultBMSSerialCommsDB = 0;
  faultBMSOVDB = 0;
  faultBMSUVDB = 0;
  faultBMSOTDB = 0;
  faultBMSUTDB = 0;
  fault12VBatOVDB = 0;
  fault12VBatUVDB = 0;
  faultWatSen1DB = 0;
  faultWatSen2DB = 0;
  faultHeatLoopDB = 0;

  //faults time stamps (TS)
  faultModuleLoopTS = 0;
  faultCANbusTS = 0;
  faultBMSSerialCommsTS = 0;
  faultBMSOVTS = 0;
  faultBMSUVTS = 0;
  faultBMSOTTS = 0;
  faultBMSUTTS = 0;
  fault12VBatOVTS = 0;
  fault12VBatUVTS = 0;
  faultWatSen1TS = 0;
  faultWatSen2TS = 0;
  faultHeatLoopTS = 0;

  isFaulted = false;
  stickyFaulted = false;

  chargerInhibit = false;
  dischargeInhibit = false;
  dc2dcON_H = false;
  heatingON_H = false;
  period = LOOP_PERIOD_ACTIVE_MS;
  bat12vVoltage = 0;

  bms.renumberBoardIDs();
  bms.clearFaults();

  initialized = true;
}

/////////////////////////////////////////////////
/// \brief This helper function allows mimicking an open collector output (floating or ground).
/////////////////////////////////////////////////
void Controller::setOutput(int pin, int state){
  if (state == 1) {
    pinMode(pin, INPUT);
  } else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state);
  }
}
/////////////////////////////////////////////////
/// \brief standby state is when the vehicle is not charging and not in run state.
///
/// Still, the vehicle or the OBC is switched on (otherwise the BMS is not powered)
/////////////////////////////////////////////////
void Controller::standby() {
  balanceCells();
  setOutput(OUTH_OBC_ON, LOW); // no charging
  setOutput(OUTH_RUN, LOW); // no running
  setOutput(OUTH_12V_BAT_CHRG, dc2dcON_H); // may charge the 12V battery
  setOutput(OUTH_BAT_HEATER, heatingON_H); // may heat the pack
  setOutput(OUTL_VALVE_OPEN, heatingON_H);
  if (heatingON_H) {
    analogWrite(OUTPWM_PUMP, 255);
  } else {
    analogWrite(OUTPWM_PUMP, 0);
  }
}

/////////////////////////////////////////////////
/// \brief pre_charge state is turning on battery heating and switches into charging, when done
/////////////////////////////////////////////////
void Controller::pre_charge() {
  balanceCells();
  setOutput(OUTH_OBC_ON, LOW); // no charging
  setOutput(OUTH_RUN, LOW); // no running
  setOutput(OUTH_12V_BAT_CHRG, dc2dcON_H); // may charge the 12V battery
  setOutput(OUTH_BAT_HEATER, heatingON_H); // may heat the pack
  setOutput(OUTL_VALVE_OPEN, heatingON_H);
  if (heatingON_H) {
    analogWrite(OUTPWM_PUMP, 255);
  } else {
    analogWrite(OUTPWM_PUMP, 0);
  }
}

/////////////////////////////////////////////////
/// \brief charging state allows the OBC to charge the pack
/////////////////////////////////////////////////
void Controller::charging() {
  balanceCells();
  setOutput(OUTH_OBC_ON, chargerInhibit);
  setOutput(OUTH_RUN, LOW); // no running
  setOutput(OUTH_12V_BAT_CHRG, LOW); // 12V charging switched off
  setOutput(OUTH_BAT_HEATER, LOW); // switch off battery heating
  setOutput(OUTL_VALVE_OPEN, LOW);
  analogWrite(OUTPWM_PUMP, (uint8_t) (getCoolingPumpDuty(bms.getHighTemperature()) * 255 ));
}

/////////////////////////////////////////////////
/// \brief post_charge state is common in BMS-OBC communication. Only balances cells
/////////////////////////////////////////////////
void Controller::post_charge() {
  balanceCells();
  setOutput(OUTH_OBC_ON, LOW);
  setOutput(OUTH_RUN, LOW);
  setOutput(OUTH_12V_BAT_CHRG, dc2dcON_H);
  setOutput(OUTH_BAT_HEATER, LOW); // switch off battery heating
  setOutput(OUTL_VALVE_OPEN, LOW);
  analogWrite(OUTPWM_PUMP, 0);
}

/////////////////////////////////////////////////
/// \brief run state is turned on and ready to operate.
/////////////////////////////////////////////////
void Controller::run() {
  setOutput(OUTH_OBC_ON, LOW);
  setOutput(OUTH_RUN, dischargeInhibit);
  setOutput(OUTH_12V_BAT_CHRG, dc2dcON_H);
  setOutput(OUTH_BAT_HEATER, LOW); // switch off battery heating
  setOutput(OUTL_VALVE_OPEN, LOW);
  analogWrite(OUTPWM_PUMP, (uint8_t) (getCoolingPumpDuty(bms.getHighTemperature()) * 255 ));
}

/////////////////////////////////////////////////
/// \brief returns the current state the controller is in.
/////////////////////////////////////////////////
Controller::ControllerState Controller::getState() {
  return state;
}

/////////////////////////////////////////////////
/// \brief returns the BMS instance to allow access to its members for reporting purposes.
/////////////////////////////////////////////////
BMSModuleManager* Controller::getBMSPtr() {
  return &bms;
}

/////////////////////////////////////////////////
/// \brief returns the main loop period the controller is expecting.
/////////////////////////////////////////////////
uint32_t Controller::getPeriodMillis() {
  return period;
}

void Controller::printControllerState() {
  uint32_t seconds = millis() / 1000;
  LOG_CONSOLE("====================================================================================\n");
  LOG_CONSOLE("=                     BMS Controller registered faults                             =\n");
  switch (state) {
    case INIT:
      LOG_CONSOLE("=  state: INIT                                                                     =\n");
      break;
    case STANDBY:
      LOG_CONSOLE("=  state: STANDBY                                                                  =\n");
      break;
    case PRE_CHARGE:
      LOG_CONSOLE("=  state: PRE_CHARGE                                                               =\n");
      break;
    case CHARGING:
      LOG_CONSOLE("=  state: CHARGING                                                                 =\n");
      break;
    case POST_CHARGE:
      LOG_CONSOLE("=  state: POST_CHARGE                                                              =\n");
      break;
    case RUN:
      LOG_CONSOLE("=  state: RUN                                                                      =\n");
      break;
  }
  LOG_CONSOLE("=  Time since last reset:%-3d days, %02d:%02d:%02d                                        =\n",
              seconds / 86400, (seconds % 86400) / 3600, (seconds % 3600) / 60, (seconds % 60));
  LOG_CONSOLE("====================================================================================\n");
  LOG_CONSOLE("%-22s   last fault time\n", "Fault Name");
  LOG_CONSOLE("----------------------   -----------------------------------------------------------\n");
  if (sFaultModuleLoop) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                      "faultModuleLoop", faultModuleLoopTS / 86400, (faultModuleLoopTS % 86400) / 3600, (faultModuleLoopTS % 3600) / 60, (faultModuleLoopTS % 60));
  if (sFaultCANbus) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                  "faultCANbus", faultCANbusTS / 86400, (faultCANbusTS % 86400) / 3600, (faultCANbusTS % 3600) / 60, (faultCANbusTS % 60));
  if (sFaultBMSSerialComms) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                          "faultBMSSerialComms", faultBMSSerialCommsTS / 86400, (faultBMSSerialCommsTS % 86400) / 3600, (faultBMSSerialCommsTS % 3600) / 60, (faultBMSSerialCommsTS % 60));
  if (sFaultBMSOV) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                 "faultBMSOV", faultBMSOVTS / 86400, (faultBMSOVTS % 86400) / 3600, (faultBMSOVTS % 3600) / 60, (faultBMSOVTS % 60));
  if (sFaultBMSUV) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                 "faultBMSUV", faultBMSUVTS / 86400, (faultBMSUVTS % 86400) / 3600, (faultBMSUVTS % 3600) / 60, (faultBMSUVTS % 60));
  if (sFaultBMSOT) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                 "faultBMSOT", faultBMSOTTS / 86400, (faultBMSOTTS % 86400) / 3600, (faultBMSOTTS % 3600) / 60, (faultBMSOTTS % 60));
  if (sFaultBMSUT) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                 "faultBMSUT", faultBMSUTTS / 86400, (faultBMSUTTS % 86400) / 3600, (faultBMSUTTS % 3600) / 60, (faultBMSUTTS % 60));
  if (sFault12VBatOV) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                    "fault12VBatOV", fault12VBatOVTS / 86400, (fault12VBatOVTS % 86400) / 3600, (fault12VBatOVTS % 3600) / 60, (fault12VBatOVTS % 60));
  if (sFault12VBatUV) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                    "fault12VBatUV", fault12VBatUVTS / 86400, (fault12VBatUVTS % 86400) / 3600, (fault12VBatUVTS % 3600) / 60, (fault12VBatUVTS % 60));
  if (sFaultWatSen1) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                   "faultWatSen1", faultWatSen1TS / 86400, (faultWatSen1TS % 86400) / 3600, (faultWatSen1TS % 3600) / 60, (faultWatSen1TS % 60));
  if (sFaultWatSen2) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                   "faultWatSen2", faultWatSen2TS / 86400, (faultWatSen2TS % 86400) / 3600, (faultWatSen2TS % 3600) / 60, (faultWatSen2TS % 60));
  if (sFaultHeatLoop) LOG_CONSOLE("%-22s @ %-3d days, %02d:%02d:%02d\n",
                                   "faultHeatLoop", faultHeatLoopTS / 86400, (faultHeatLoopTS % 86400) / 3600, (faultHeatLoopTS % 3600) / 60, (faultHeatLoopTS % 60));
}
