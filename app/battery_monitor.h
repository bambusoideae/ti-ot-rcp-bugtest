/*
 * battery_monitor.h
 *
 *  Created on: Jul 15, 2023
 *      Author: mushu
 */

#ifndef APP_BATTERY_MONITOR_H_
#define APP_BATTERY_MONITOR_H_

// Begin C scope
#ifdef __cplusplus
extern "C"
{
#endif

// Include libraries

// Constants

// Data types

// Public prototypes
void enableBatteryMonitor();
uint32_t readBatteryVoltage();
int32_t readChipTemperature();

// End of C scope
#ifdef __cplusplus
}
#endif


#endif /* APP_BATTERY_MONITOR_H_ */
