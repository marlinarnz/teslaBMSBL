#include "CanManager.hpp"

/////////////////////////////////////////////////
/// \brief Coordinates reading and writing messages to the CAN bus
/////////////////////////////////////////////////
void CanManager::doCan() {
  // Get the controller state and act, if it's active
  Controller::ControllerState state = controller_inst_ptr->getState();
  if (state != Controller::INIT){
    bool success = true;
    success &= writeToOBC_MitsubishiOutlander(state);
    success &= writeToVCU();
    success &= read();
    controller_inst_ptr->reportCanStatus(success);
  }
}

/////////////////////////////////////////////////
/// \brief Constructor sets up the messages
/////////////////////////////////////////////////
CanManager::CanManager(Controller* c, int intervalVCU, int intervalOBC)
  : controller_inst_ptr(c),
    sendIntervalVCU(intervalVCU), lastSentTimeVCU(0),
    sendIntervalOBC(intervalOBC), lastSentTimeOBC(0)
{}

/////////////////////////////////////////////////
/// \brief Destructor terminates the CAN bus
/////////////////////////////////////////////////
CanManager::~CanManager() {
  Can0.end();
}

/////////////////////////////////////////////////
/// \brief Initialise CAN bus communication
/////////////////////////////////////////////////
void CanManager::init() {
  // begin communication and set a recieve mask for extended message IDs
  // It is possible to set more filters using setFilter(const CAN_filter_t &filter, uint8_t n);
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 1;
  mask.id = 0;
  Can0.setNumTxBoxes(4);
  Can0.begin(CANBUS_SPEED);
  Can0.startStats();
  LOG_INFO("Started CAN bus communication");
}

/////////////////////////////////////////////////
/// \brief Report BMS status to the vehicle
/////////////////////////////////////////////////
bool CanManager::writeToVCU() {
  if (millis() - lastSentTimeVCU >= sendIntervalVCU) {
    // Instantiate a new message
    CAN_message_t writeMsg;
    writeMsg.id = 0x103;
    writeMsg.len = 8;
    writeMsg.flags.extended = 0;
    writeMsg.flags.remote = 0;

    // Write the message to the vehicle
    writeMsg.buf[0] = highByte(uint16_t(controller_inst_ptr->getBMSPtr()->getPackVoltage() * 10));
    writeMsg.buf[1] = lowByte(uint16_t(controller_inst_ptr->getBMSPtr()->getPackVoltage() * 10));
    writeMsg.buf[2] = uint8_t(controller_inst_ptr->getBMSPtr()->getAvgTemperature() + 40);
    writeMsg.buf[3] = uint8_t(controller_inst_ptr->getBMSPtr()->getHighTemperature() + 40);
    writeMsg.buf[4] = uint8_t(controller_inst_ptr->getBMSPtr()->getLowTemperature() + 40);
    writeMsg.buf[5] = highByte(uint8_t(controller_inst_ptr->bat12vVoltage * 10));
    writeMsg.buf[6] = (uint8_t(controller_inst_ptr->getState()) << 4)
                      | (uint8_t(controller_inst_ptr->isFaulted) << 3)
                      | (uint8_t(controller_inst_ptr->stickyFaulted) << 2)
                      | (uint8_t(controller_inst_ptr->chargerInhibit) << 1)
                      | uint8_t(controller_inst_ptr->dischargeInhibit);
    writeMsg.buf[7] = (uint8_t(controller_inst_ptr->faultModuleLoop || controller_inst_ptr->faultBMSSerialComms) << 7)
                      | (uint8_t(controller_inst_ptr->faultBMSOV) << 6)
                      | (uint8_t(controller_inst_ptr->faultBMSUV) << 5)
                      | (uint8_t(controller_inst_ptr->faultBMSOT) << 4)
                      | (uint8_t(controller_inst_ptr->faultBMSUT) << 3)
                      | (uint8_t(controller_inst_ptr->fault12VBatOV || controller_inst_ptr->fault12VBatUV) << 2)
                      | (uint8_t(controller_inst_ptr->faultWatSen1 || controller_inst_ptr->faultWatSen2) << 1)
                      | uint8_t(controller_inst_ptr->faultHeatLoop);
  
    lastSentTimeVCU = millis();
    return (bool)(Can0.write(writeMsg));
  }
  return true;
}

/////////////////////////////////////////////////
/// \brief Give commands to an Elcon-like OBC via CAN bus. Returns false in case of an error
/////////////////////////////////////////////////
bool CanManager::writeToOBC_Elcon() {
  if (millis() - lastSentTimeOBC >= sendIntervalOBC) {
    // Instantiate a new message
    CAN_message_t writeMsg;
    writeMsg.id = 0x1806E5F4;
    writeMsg.len = 8;
    writeMsg.flags.extended = 1;
    writeMsg.flags.remote = 0;

    // Get the controller state and derive the charging command from it
    // 0: Charger on, charging
    // 1: Battery protection, charger output off
    // 2: Heating mode on
    // 3: Charging complete
    uint8_t commandToCharger;
    switch (controller_inst_ptr->getState()) {
      case Controller::PRE_CHARGE:
        commandToCharger = 2;
        break;
      case Controller::CHARGING:
        commandToCharger = 0;
        break;
      case Controller::POST_CHARGE:
        commandToCharger = 3;
        break;
      default:
        commandToCharger = 1;
        break;
    }

    // Get the desired pack voltage
    float chargeVoltage = MAX_CHARGE_V_SETPOINT * controller_inst_ptr->getBMSPtr()->seriesCells();

    // Calculate the charge amperage for each charger
    float chargeAmperage = MAX_CHARGE_A_SETPOINT / N_CHARGERS;
    // Regulate based on SOC and cell temperatures
    chargeAmperage = regulateChargeAmperage(chargeAmperage);
    
    // Write the message to the OBC (based on Elcon CAN matrix)
    writeMsg.buf[0] = highByte(uint16_t(chargeVoltage * 10));
    writeMsg.buf[1] = lowByte(uint16_t(chargeVoltage * 10));
    writeMsg.buf[2] = highByte(uint16_t(chargeAmperage * 10));
    writeMsg.buf[3] = lowByte(uint16_t(chargeAmperage * 10));
    writeMsg.buf[4] = commandToCharger;
    writeMsg.buf[5] = 0x00;
    writeMsg.buf[6] = 0x00;
    writeMsg.buf[7] = 0x00;
  
    lastSentTimeOBC = millis();
    return (bool)(Can0.write(writeMsg));
  }
  return true;
}

/////////////////////////////////////////////////
/// \brief Give commands to an Mitsubishi Outlander OBC via CAN bus. Returns false in case of an error
/////////////////////////////////////////////////
bool CanManager::writeToOBC_MitsubishiOutlander(Controller::ControllerState state) {
  
  // Activate the charger
  if (state == Controller::PRE_CHARGE) {
    if (millis() - lastSentTimeOBC >= sendIntervalOBC) {
      // Instantiate a new message
      CAN_message_t writeMsg;
      writeMsg.id = 0x285;
      writeMsg.len = 8;
      writeMsg.flags.extended = 0;
      writeMsg.flags.remote = 0;
      writeMsg.buf[0] = 0x00;
      writeMsg.buf[1] = 0x00;
      writeMsg.buf[2] = 0xB6;
      writeMsg.buf[3] = 0x00;
      writeMsg.buf[4] = 0x00;
      writeMsg.buf[5] = 0x00;
      writeMsg.buf[6] = 0x00;
      writeMsg.buf[7] = 0x00;
    
      lastSentTimeOBC = millis();
      return (bool)(Can0.write(writeMsg));
    }
  }

  // Let it charge
  else if (state == Controller::CHARGING) {
    if (millis() - lastSentTimeOBC >= sendIntervalOBC) {
      // Instantiate a new message
      CAN_message_t writeMsg;
      writeMsg.id = 0x286;
      writeMsg.len = 8;
      writeMsg.flags.extended = 0;
      writeMsg.flags.remote = 0;

      // Get the desired pack voltage
      float chargeVoltage = MAX_CHARGE_V_SETPOINT * controller_inst_ptr->getBMSPtr()->seriesCells();

      // Calculate the charge amperage for each charger
      float chargeAmperage = MAX_CHARGE_A_SETPOINT / N_CHARGERS;
      // The maximum for the Mitsubishi Outlander OBC is 12A
      if (chargeAmperage > 12) chargeAmperage = 12.0f;
      // Regulate based on SOC and cell temperatures
      chargeAmperage = regulateChargeAmperage(chargeAmperage);
      
      // Write the message to the OBC
      writeMsg.buf[0] = highByte(uint16_t(chargeVoltage * 10));
      writeMsg.buf[1] = lowByte(uint16_t(chargeVoltage * 10));
      writeMsg.buf[2] = uint8_t(chargeAmperage * 10);
      writeMsg.buf[3] = 0x37;
      writeMsg.buf[4] = 0x00;
      writeMsg.buf[5] = 0x00;
      writeMsg.buf[6] = 0x0A;
      writeMsg.buf[7] = 0x00;
    
      lastSentTimeOBC = millis();
      return (bool)(Can0.write(writeMsg));
    }
  }
  return true;
}

/////////////////////////////////////////////////
/// \brief Reduce the maximum charge amperage if close to maximum charge or high/low temperatures
/////////////////////////////////////////////////
float CanManager::regulateChargeAmperage(float maxAmp) {

  // When cells are too cold, reduce charge amperage
  float lowCellTemp = controller_inst_ptr->getBMSPtr()->getLowTemperature();
  if (lowCellTemp < HEATING_T_SETPOINT) {
    maxAmp -= (HEATING_T_SETPOINT - lowCellTemp) * CHARGE_A_REDUCTION_FACTOR_LOWTEMP;
  }

  // When cells are too hot, reduce charge amperage
  float highCellTemp = controller_inst_ptr->getBMSPtr()->getHighTemperature();
  if (highCellTemp > COOLING_HIGHT_SETPOINT) {
    maxAmp -= (highCellTemp - COOLING_HIGHT_SETPOINT) * CHARGE_A_REDUCTION_FACTOR_HIGHTEMP;
  }

  // When the cells come close to max charge, reduce charge amperage
  float avgCellVolt = controller_inst_ptr->getBMSPtr()->getAvgCellVolt();
  if (avgCellVolt > PRECISION_BALANCE_V_SETPOINT) {
    maxAmp -= (avgCellVolt - PRECISION_BALANCE_V_SETPOINT) * CHARGE_A_REDUCTION_FACTOR_VOLT;
  }

  // Charge amperage should not lie below a trickle charge threshold
  if (maxAmp <= MIN_CHARGE_A_SETPOINT) maxAmp = MIN_CHARGE_A_SETPOINT;

  // Conditions that prevent charging
  if (controller_inst_ptr->chargerInhibit
      || avgCellVolt > MAX_CHARGE_V_SETPOINT) {
    maxAmp = 0;
  }

  return maxAmp;
}

/////////////////////////////////////////////////
/// \brief Read message(s) from the CAN bus. Returns false in case of an error
/////////////////////////////////////////////////
bool CanManager::read() {
  // Check if a new message is available and read as long as there is a frame waiting
  while (Can0.available()) {
    CAN_message_t readMsg;
    if (Can0.read(readMsg)) {
      
      // Go through all known message IDs and react accordingly
      switch (readMsg.id) {
        case 0x18FF50E5:
          break;
        case 0x18FF50E6:
          break;
        case 0x18FF50E7:
          break;
        default:
          break;
      }
    } else {
      // A message is available, but couldn't be read
      LOG_ERROR("Error while trying to read the CAN bus");
      CAN_stats_t stats = Can0.getStats();
      if (stats.enabled) {
        if (stats.ringRxFramesLost > 0) {
          LOG_WARN("Lost frames on the CAN bus since last error");
          Can0.clearStats();
        }
      }
      return false;
    }
  }
  return true;
}
