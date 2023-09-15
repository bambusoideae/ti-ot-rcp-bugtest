/*
 * battery_monitor.c
 *
 *  Created on: Jul 15, 2023
 *      Author: mushu
 */

// Include libraries
/* driverlib specific header */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)

// Declare function
void enableBatteryMonitor() {
    AONBatMonEnable();
}

// Read BAT Voltage
uint32_t readBatteryVoltage() {
    // AONBatMonNewBatteryMeasureReady // Non-blocking (check & clear)
    // voltage = (voltage * 1000) >> AON_BATMON_BAT_FRAC_W;
    uint32_t milivolts = (AONBatMonBatteryVoltageGet()*125)>>5;
    return milivolts;
}

// Read Internal chip Temperature
int32_t readChipTemperature() {
    // AONBatMonNewTempMeasureReady // Non-blocking (check & clear)
    int32_t degC = AONBatMonTemperatureGetDegC();
    return degC;
}

