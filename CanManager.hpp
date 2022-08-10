#include <Arduino.h>
#include "CONFIG.h"
#include "Controller.hpp"
#include "Logger.hpp"
#include <FlexCAN.h> // Semms to use collin80's library: https://github.com/collin80/FlexCAN_Library

#ifndef CANMANAGER_HPP_
#define CANMANAGER_HPP_

class CanManager {
  public:
    void doCan();
    CanManager(Controller* c_ptr);
    ~CanManager();
    void init();

  private:
    Controller* controller_inst_ptr;
    bool read();
    CAN_message_t readMsg;
    bool writeToOBC();
    CAN_message_t writeMsg;
    int sendInterval;
    long lastSentTime;

};

#endif /* CANMANAGER_H_ */
