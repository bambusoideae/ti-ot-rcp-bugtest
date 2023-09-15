/*
 * app.h
 *
 *  Created on: Jun 23, 2023
 *      Author: mushu
 */

#ifndef APP_APP_H_
#define APP_APP_H_

// Begin C scope
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/**
 * Application events.
 */
typedef enum
{
    App_evtNwkSetup,              /* OpenThread network is setup */
    App_evtKeyRight,              /* Right key is pressed */
    App_evtNwkJoined,             /* Joined the network */
    App_evtNwkJoinFailure,        /* Failed joining network */
    App_evtDevRoleChanged,        /* Events for Device State */
    App_evtMonitorTrigger,         /* Monitor timer is trigger */
#if TIOP_CUI
    App_evtNwkAttach,             /* CUI Menu Attach option is selected */
    App_evtNwkJoin,               /* CUI Menu Join option is selected */
    App_evtProcessMenuUpdate,     /* CUI Menu Event is triggered */
#endif /* TIOP_CUI */
} App_evt;

/******************************************************************************
 External functions
 *****************************************************************************/

/**
 * @brief Posts an event to the App task.
 *
 * @param event event to post.
 * @return None
 */
extern void app_postEvt(App_evt event);

// End of C scope
#ifdef __cplusplus
}
#endif

#endif /* APP_APP_H_ */
