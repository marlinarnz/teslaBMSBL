#include <Arduino.h>
#include "CONFIG.h"
#include "BMSModuleManager.hpp"

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

class Controller {
  public:
    enum ControllerState {
      INIT, STANDBY, PRE_CHARGE, CHARGING, POST_CHARGE, RUN
    };
    void doController();
    Controller();
    ControllerState getState();
    BMSModuleManager* getBMSPtr();
    void printControllerState();
    uint32_t getPeriodMillis();
    void reportCanStatus(bool allGood);

    //faults
    bool faultModuleLoop;
    bool faultCANbus;
    bool faultBMSSerialComms;
    bool faultBMSOV;
    bool faultBMSUV;
    bool faultBMSOT;
    bool faultBMSUT;
    bool fault12VBatOV;
    bool fault12VBatUV;
    bool faultWatSen1;
    bool faultWatSen2;
    bool faultHeatLoop;

    //sticky faults
    bool sFaultModuleLoop;
    bool sFaultCANbus;
    bool sFaultBMSSerialComms;
    bool sFaultBMSOV;
    bool sFaultBMSUV;
    bool sFaultBMSOT;
    bool sFaultBMSUT;
    bool sFault12VBatOV;
    bool sFault12VBatUV;
    bool sFaultWatSen1;
    bool sFaultWatSen2;
    bool sFaultHeatLoop;

    //faults debounce counters (DB)
    uint8_t faultModuleLoopDB;
    uint8_t faultCANbusDB;
    uint8_t faultBMSSerialCommsDB;
    uint8_t faultBMSOVDB;
    uint8_t faultBMSUVDB;
    uint8_t faultBMSOTDB;
    uint8_t faultBMSUTDB;
    uint8_t fault12VBatOVDB;
    uint8_t fault12VBatUVDB;
    uint8_t faultWatSen1DB;
    uint8_t faultWatSen2DB;
    uint8_t faultHeatLoopDB;

    //faults time stamps (TS)
    uint32_t faultModuleLoopTS;
    uint32_t faultCANbusTS;
    uint32_t faultBMSSerialCommsTS;
    uint32_t faultBMSOVTS;
    uint32_t faultBMSUVTS;
    uint32_t faultBMSOTTS;
    uint32_t faultBMSUTTS;
    uint32_t fault12VBatOVTS;
    uint32_t fault12VBatUVTS;
    uint32_t faultWatSen1TS;
    uint32_t faultWatSen2TS;
    uint32_t faultHeatLoopTS;

    bool isFaulted;
    bool stickyFaulted;
    bool chargerInhibit;
    bool dischargeInhibit;
    float bat12vVoltage;

  private:

    BMSModuleManager bms;

    bool dc2dcON_H;
    bool heatingON_H;
    uint32_t period;

    //run-time functions
    void syncModuleDataObjects(); //gathers all the data from the boards and populates the BMSModel object instances
    void balanceCells(); //balances the cells according to thresholds in the BMSModuleManager

    void assertFaultLine();
    void clearFaultLine();
    float getCoolingPumpDuty(float);

    ControllerState state;
    bool initialized;
    void setOutput(int pin, int state);
    void init(); //reset all boards and assign address to each board
    void standby();
    void pre_charge();
    void charging();
    void post_charge();
    void run();

};

#endif /* CONTROLLER_H_ */
