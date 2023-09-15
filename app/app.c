/*
 * app.c
 *
 *  Created on: Jun 23, 2023
 *      Author: mushu
 */
/******************************************************************************
 ******************************************************************************

 Copyright (c) 2017-2023, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/
#include <openthread/config.h>
#include <openthread-core-config.h>

/* Standard Library Header files */
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* POSIX Header files */
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <mqueue.h>

/* OpenThread public API Header files */
#include <openthread/dataset.h>
#include <openthread/platform/logging.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>
#include <openthread/coap.h>
#include <openthread/ip6.h>
#include <openthread/ping_sender.h>
#include <openthread/link.h>

/* driverlib specific header */
#include <ti/devices/DeviceFamily.h>

#include <utils/uart.h>

/* OpenThread Internal/Example Header files */
#include "otsupport/otrtosapi.h"
#include "otsupport/otinstance.h"

/* RTOS Header files */
#include <ti/sysbios/runtime/System.h>
// #include <ti/sysbios/runtime/Error.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/GPIO.h>

/* Example/Board Header files */
#include "ti_drivers_config.h"

#include "app.h"
#include "battery_monitor.h"

#include "utils/code_utils.h"
#include "disp_utils.h"
#include "otstack.h"

/* Private configuration Header files */
#include "task_config.h"
#include "tiop_config.h"

#if TIOP_CUI
#include "cui.h"
#include "tiop_ui.h"
#endif


// Error assert
#if (OPENTHREAD_CONFIG_COAP_API_ENABLE == 0)
#error "OPENTHREAD_CONFIG_COAP_API_ENABLE needs to be defined and set to 1"
#endif

/******************************************************************************
 Constants and definitions
 *****************************************************************************/
#define APP_PROC_QUEUE_MAX_MSG (10)

struct App_procQueueMsg
{
    App_evt evt;
};

/******************************************************************************
 Local variables
 *****************************************************************************/

/* Timer ID */
// static timer_t reportTimerID;
static char monitorAttr[20] = "3100mV; 30T";
static char reportBody[255] = "";
static char monitorUri[5] = "m";
static int rCount = 0;

// interface ip
static char ifip[OT_IP6_ADDRESS_STRING_SIZE + 1] = "";

/* Server IPv6 address */
static otIp6Address serverAddress;

/* Server COAP Port */
static uint16_t serverPort = OT_DEFAULT_COAP_PORT;

/* POSIX message queue for passing events to the application processing loop. */
const  char  App_procQueueName[] = "/em_process";
static mqd_t App_procQueueDesc;

/* OpenThread Stack thread call stack */
static char stack[TASK_CONFIG_APP_TASK_STACK_SIZE];

#if TIOP_CUI
/* String variable for copying over app lines to CUI */
static char statusBuf[MAX_STATUS_LINE_VALUE_LEN] = "[" CUI_COLOR_CYAN "Empty State" CUI_COLOR_RESET "] ";
static Button_Handle rightButtonHandle;
#endif

/******************************************************************************
 Function Prototype
 *****************************************************************************/


/* Application thread */
void *app_task(void *arg0);
void monitorEventHandler();

/******************************************************************************
 Local Functions
 *****************************************************************************/

/**
 * @brief Processes the OT stack events
 *
 * @param event             event identifier
 * @param aContext          context pointer for the event
 *
 * @return None
 */
static void processOtStackEvents(uint8_t event, void *aContext)
{
    (void) aContext;

    switch (event)
    {
        case OT_STACK_EVENT_NWK_JOINED:
        {
            app_postEvt(App_evtNwkJoined);
            break;
        }

        case OT_STACK_EVENT_NWK_JOINED_FAILURE:
        {
            app_postEvt(App_evtNwkJoinFailure);
            break;
        }

        case OT_STACK_EVENT_NWK_DATA_CHANGED:
        {
            app_postEvt(App_evtNwkSetup);
            break;
        }

        case OT_STACK_EVENT_DEV_ROLE_CHANGED:
        {
            app_postEvt(App_evtDevRoleChanged);
            break;
        }

        default:
        {
            break;
        }
    }
}


/**
 * @brief Processes the events.
 *
 * @return None
 */
static void processEvent(App_evt event)
{
    switch (event)
    {
        case App_evtKeyRight:
        {
            if ((!otDatasetIsCommissioned(OtInstance_get())) &&
                    (OtStack_joinState() != OT_STACK_EVENT_NWK_JOIN_IN_PROGRESS))
            {
                DISPUTILS_SERIALPRINTF(1, 0, "Joining Nwk ...");
                OtStack_joinConfiguredNetwork();
            }
            break;
        }

        case App_evtNwkSetup:
            // setupMonitorCoapServer(OtInstance_get());
            break;

        case App_evtNwkJoined:
        {
            DISPUTILS_SERIALPRINTF( 1, 0, "Joined Nwk");
            (void)OtStack_setupNetwork();
            break;
        }

        case App_evtNwkJoinFailure:
        {
            DISPUTILS_SERIALPRINTF(1, 0, "Join Failure");
            break;
        }

        case App_evtMonitorTrigger:
        {
            monitorEventHandler();
            break;
        }

        default:
        {
            break;
        }
    }

#if TIOP_CUI
    /* Update the UI */
    switch (event)
    {
        case App_evtKeyRight:
        {
            if ((!otDatasetIsCommissioned(OtInstance_get())) &&
                (OtStack_joinState() != OT_STACK_EVENT_NWK_JOIN_IN_PROGRESS))
            {
                tiopCUIUpdateConnStatus(CUI_conn_joining);
            }
            break;
        }

        case App_evtNwkAttach:
        {
            DISPUTILS_LCDPRINTF(1, 0, "Joining Nwk ...");
            tiopCUIUpdateConnStatus(CUI_conn_joining);
            (void)OtStack_setupInterfaceAndNetwork();
            break;
        }

        case App_evtNwkJoin:
        {
            if ((!otDatasetIsCommissioned(OtInstance_get())) &&
                (OtStack_joinState() != OT_STACK_EVENT_NWK_JOIN_IN_PROGRESS))
            {
                DISPUTILS_LCDPRINTF(1, 0, "Joining Nwk ...");
                tiopCUIUpdateConnStatus(CUI_conn_joining);
                OtStack_joinConfiguredNetwork();
                break;
            }
        }

        case App_evtNwkJoined:
        {
            otNetworkKey networkKey;
            OtRtosApi_lock();
            tiopCUIUpdatePANID(otLinkGetPanId(OtInstance_get()));
            tiopCUIUpdateChannel(otLinkGetChannel(OtInstance_get()));
            tiopCUIUpdateShortAddr(otLinkGetShortAddress(OtInstance_get()));
            tiopCUIUpdateNwkName(otThreadGetNetworkName(OtInstance_get()));
            otThreadGetNetworkKey(OtInstance_get(), &networkKey);
            tiopCUIUpdateNetworkKey(networkKey);
            tiopCUIUpdateExtPANID(*(otThreadGetExtendedPanId(OtInstance_get())));
            OtRtosApi_unlock();
            break;
        }

        case App_evtNwkJoinFailure:
        {
            tiopCUIUpdateConnStatus(CUI_conn_join_fail);
            break;
        }

        case App_evtProcessMenuUpdate:
        {
            CUI_processMenuUpdate();
            break;
        }

        case App_evtDevRoleChanged:
        {
            OtRtosApi_lock();
            otDeviceRole role = otThreadGetDeviceRole(OtInstance_get());
            OtRtosApi_unlock();

            tiopCUIUpdateRole(role);
            switch (role)
            {
                case OT_DEVICE_ROLE_DISABLED:
                case OT_DEVICE_ROLE_DETACHED:
                    break;

                case OT_DEVICE_ROLE_CHILD:
                case OT_DEVICE_ROLE_ROUTER:
                case OT_DEVICE_ROLE_LEADER:
                {
                    otNetworkKey networkKey;
                    tiopCUIUpdateConnStatus(CUI_conn_joined);

                    OtRtosApi_lock();
                    tiopCUIUpdatePANID(otLinkGetPanId(OtInstance_get()));
                    tiopCUIUpdateChannel(otLinkGetChannel(OtInstance_get()));
                    tiopCUIUpdateShortAddr(otLinkGetShortAddress(OtInstance_get()));
                    tiopCUIUpdateNwkName(otThreadGetNetworkName(OtInstance_get()));
                    otThreadGetNetworkKey(OtInstance_get(), &networkKey);
                    tiopCUIUpdateNetworkKey(networkKey);
                    tiopCUIUpdateExtPANID(*(otThreadGetExtendedPanId(OtInstance_get())));
                    OtRtosApi_unlock();
                    break;
                }

                default:
                {
                    break;
                }
            }
        }

        default:
        {
            break;
        }
    }
#endif /* TIOP_CUI */
}

// Monitor timer handler
static void monitorTimerHandler(uintptr_t arg1) {
    app_postEvt(App_evtMonitorTrigger);
}

// Setup monitor Timer
static void createMonitorTimer() {
    Clock_Params clockParams;
    Clock_Handle monitorClock;
    Clock_Params_init(&clockParams);
    clockParams.period = 200000; // 2 seconds (clock tick is 10us) - Clock_tickPeriod_D
    clockParams.startFlag = true;
    monitorClock = Clock_create(&monitorTimerHandler, 200000, &clockParams, Error_IGNORE);
    if (monitorClock == NULL) {
        System_abort("Monitor clock create failed!");
    }
}


void updateIpaddress() {
    // GET Interface IP address
    const otNetifAddress *unicastAddrs = otIp6GetUnicastAddresses(OtInstance_get());
    int n = 0;
    for (const otNetifAddress *addr = unicastAddrs; addr; addr = addr->mNext) {
        n++;
        otIp6AddressToString(&(addr->mAddress), ifip, OT_IP6_ADDRESS_STRING_SIZE);
        if (n == 1) {
            tiopCUIUpdateServer(ifip);
        } else if (n == 2) {
            tiopCUIUpdateServer2(ifip);
        }
    }
    tiopCUIUpdateSensor(ifip);
}

void monitorEventHandler() {
    // Update interface info in UI
    updateIpaddress();

    // Read BAT Voltage
    uint32_t batMiliVoltage = readBatteryVoltage();
    int32_t chipTemperature = readChipTemperature();
    rCount++;

    // COAP Body
    snprintf((char*)monitorAttr, sizeof(monitorAttr), "%dmV; %d°C | %d |", batMiliVoltage, chipTemperature, rCount);
    tiopCUIUpdateApp(monitorAttr); // CUI
    snprintf((char*)reportBody, sizeof(reportBody),
             "%dmV; %d°C | %d | dummy payload - Form a network with the device that has Commissioner support. - Mesh Local Prefix: fd3d:b50b:f96d:722d::/64",
             batMiliVoltage, chipTemperature, rCount);
    coapRequest(&serverAddress, serverPort, monitorUri, reportBody); // POST COAP Request
}

/******************************************************************************
 External Functions
 *****************************************************************************/

/**
 * Documented in openthread/platform/uart.h.
 */
void otPlatUartReceived(const uint8_t *aBuf, uint16_t aBufLength)
{
    (void)aBuf;
    (void)aBufLength;
    /* Do nothing. */
}

/**
 * Documented in openthread/platform/uart.h.
 */
void otPlatUartSendDone(void)
{
    /* Do nothing. */
}

/* Documented in app.h */
void app_postEvt(App_evt event)
{
    struct App_procQueueMsg msg;
    int ret;
    msg.evt = event;
    ret = mq_send(App_procQueueDesc, (const char *)&msg, sizeof(msg), 0);
    assert(0 == ret);
    (void)ret;
}


#if TIOP_CUI
/**
 * @brief Handles the key press events.
 *
 * @param _buttonHandle identifies which keys were pressed
 * @param _buttonEvents identifies the event that occurred on the key
 * @return None
 */
void processKeyChangeCB(Button_Handle _buttonHandle, Button_EventMask _buttonEvents)
{
    if (_buttonHandle == rightButtonHandle && _buttonEvents & Button_EV_CLICKED)
    {
        app_postEvt(App_evtKeyRight);
    }
}

/**
 * documented in tiop_ui.h
 */
void processMenuUpdateFn(void)
{
    app_postEvt(App_evtProcessMenuUpdate);
}

/**
 * documented in tiop_ui.h
 */
void tiopCUINwkReset(const char _input, char* _pLines[3], CUI_cursorInfo_t* _pCurInfo)
{
    if (CUI_ITEM_INTERCEPT_START == _input)
    {
        OtRtosApi_lock();
        otInstanceFactoryReset(OtInstance_get());
        OtRtosApi_unlock();
        strncpy(_pLines[2], "Resetting, Please wait...", MAX_MENU_LINE_LEN);
    }
}

/**
 * documented in tiop_ui.h
 */
void tiopCUIReset(const char _input, char* _pLines[3], CUI_cursorInfo_t* _pCurInfo)
{
    if (CUI_ITEM_INTERCEPT_START == _input)
    {
        OtRtosApi_lock();
        otInstanceReset(OtInstance_get());
        OtRtosApi_unlock();
        strncpy(_pLines[2], "Resetting, Please wait...", MAX_MENU_LINE_LEN);
    }
}

/**
 * documented in tiop_ui.h
 */
void uiActionAttach(const int32_t _itemEntry)
{
    app_postEvt(App_evtNwkAttach);
}

/**
 * documented in tiop_ui.h
 */
void uiActionJoin(const int32_t _itemEntry)
{
    app_postEvt(App_evtNwkJoin);
}
#endif /* TIOP_CUI */

/**
 * Documented in task_config.h.
 */
void app_taskCreate(void)
{
    pthread_t           thread;
    pthread_attr_t      pAttrs;
    struct sched_param  priParam;
    int                 retc;

    retc = pthread_attr_init(&pAttrs);
    assert(retc == 0);

    retc = pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    assert(retc == 0);

    priParam.sched_priority = TASK_CONFIG_APP_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    assert(retc == 0);

    retc = pthread_attr_setstack(&pAttrs, (void *)stack,
                                 TASK_CONFIG_APP_TASK_STACK_SIZE);
    assert(retc == 0);

    retc = pthread_create(&thread, &pAttrs, app_task, NULL);
    assert(retc == 0);

    retc = pthread_attr_destroy(&pAttrs);
    assert(retc == 0);

    (void) retc;
}

/**
 * Main thread starting the Empty example within OpenThread.
 */
void *app_task(void *arg0)
{
    struct mq_attr attr;
    mqd_t          procQueueLoopDesc;

    attr.mq_curmsgs = 0;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = APP_PROC_QUEUE_MAX_MSG;
    attr.mq_msgsize = sizeof(struct App_procQueueMsg);

    /* Open the processing queue in non-blocking mode for the notify
     * callback functions
     */
    App_procQueueDesc = mq_open(App_procQueueName,
                                       (O_WRONLY | O_NONBLOCK | O_CREAT),
                                       0, &attr);

    /* Open the processing queue in blocking read mode for the process loop */
    procQueueLoopDesc = mq_open(App_procQueueName, O_RDONLY, 0, NULL);

    DispUtils_open();

    OtStack_taskCreate();

    OtStack_registerCallback(processOtStackEvents);

    /* Set the poll period, as NVS does not store poll period */
    OtRtosApi_lock();
    otLinkSetPollPeriod(OtInstance_get(), TIOP_CONFIG_POLL_PERIOD);
    OtRtosApi_unlock();

#if TIOP_CUI
    tiopCUIInit(statusBuf, &rightButtonHandle);
#endif /* TIOP_CUI */

    DISPUTILS_SERIALPRINTF(0, 0, "LPSTK Initializing!");

#if TIOP_CONFIG_SET_NW_ID
    OtStack_setupInterfaceAndNetwork();
#else
    {
        bool commissioned;

        OtRtosApi_lock();
        commissioned = otDatasetIsCommissioned(OtInstance_get());
        OtRtosApi_unlock();

        if (commissioned)
        {
            OtStack_setupInterfaceAndNetwork();
        }
        else
        {
            otExtAddress extAddress;

            OtRtosApi_lock();
            otLinkGetFactoryAssignedIeeeEui64(OtInstance_get(), &extAddress);
            OtRtosApi_unlock();

            DISPUTILS_SERIALPRINTF(2, 0, "pskd: %s", TIOP_CONFIG_PSKD);
            DISPUTILS_SERIALPRINTF(3, 0, "EUI64: 0x%02x%02x%02x%02x%02x%02x%02x%02x",
                                   extAddress.m8[0], extAddress.m8[1], extAddress.m8[2],
                                   extAddress.m8[3], extAddress.m8[4], extAddress.m8[5],
                                   extAddress.m8[6], extAddress.m8[7]);

        }
    }
#endif /* !TIOP_CONFIG_SET_NW_ID */

    OtRtosApi_lock();
    // All FTD (router) - Link-Local address: ff02::2
    otIp6AddressFromString("ff03::1", &serverAddress); // All FTD (router) - Mesh-Local
    OtRtosApi_unlock();

    // Start COAP Server
    OtRtosApi_lock();
    otCoapStart(OtInstance_get(), OT_DEFAULT_COAP_PORT);
    OtRtosApi_unlock();

    // Enable the temperature and battery monitoring
    enableBatteryMonitor();
    // Setup monitor timer
    createMonitorTimer();

    while (1)
    {
        struct App_procQueueMsg msg;
        ssize_t ret;

        ret = mq_receive(procQueueLoopDesc, (char *)&msg, sizeof(msg), NULL);
        /* priorities are ignored */
        if (ret < 0 || ret != sizeof(msg))
        {
            /* something has failed */
            continue;
        }
        processEvent(msg.evt);
    }
}
