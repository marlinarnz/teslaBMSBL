#include "CanManager.hpp"

/////////////////////////////////////////////////
/// \brief Coordinates reading and writing messages to the CAN bus
/////////////////////////////////////////////////
void CanManager::doCan() {
  // Get the controller state and act, if it's active
  Controller::ControllerState state = controller_inst_ptr->getState();
  if (state != Controller::INIT){
    bool success = writeToOBC();
    success &= read();
    controller_inst_ptr->reportCanStatus(success);
  }
}

/////////////////////////////////////////////////
/// \brief Constructor sets up the messages
/////////////////////////////////////////////////
CanManager::CanManager(Controller* c)
  : controller_inst_ptr(c), sendInterval(500), lastSentTime(0)
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
  Can0.begin(CANBUS_SPEED);
  LOG_INFO("Started CAN bus communication");
}

/////////////////////////////////////////////////
/// \brief Give commands to an on-board charger via CAN bus. Returns false in case of an error
/////////////////////////////////////////////////
bool CanManager::writeToOBC() {
  if (millis() - lastSentTime >= sendInterval) {
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
    // When the cells come close to max charge, reduce charge amperage
    float chargeAmperage = MAX_CHARGE_A_SETPOINT / N_CHARGERS;
    if (controller_inst_ptr->getBMSPtr()->getAvgCellVolt() > PRECISION_BALANCE_V_SETPOINT) {
      chargeAmperage -= (controller_inst_ptr->getBMSPtr()->getAvgCellVolt() - PRECISION_BALANCE_V_SETPOINT) * CHARGE_A_REDUCTION_FACTOR;
    }
    
    // Write the message to the OBC (based on Elcon CAN matrix)
    writeMsg.buf[0] = highByte(uint16_t(chargeVoltage * 10));
    writeMsg.buf[1] = lowByte(uint16_t(chargeVoltage * 10));
    writeMsg.buf[2] = highByte(uint16_t(chargeAmperage * 10));
    writeMsg.buf[3] = lowByte(uint16_t(chargeAmperage * 10));
    writeMsg.buf[4] = commandToCharger;
    writeMsg.buf[5] = 0x00;
    writeMsg.buf[6] = 0x00;
    writeMsg.buf[7] = 0x00;
  
    lastSentTime = millis();
    return (bool)(Can0.write(writeMsg));
  }
  return true;
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
      return false;
    }
  }
  return true;
}
