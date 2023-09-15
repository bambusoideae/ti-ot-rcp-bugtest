/*
 * coap_helper.h
 *
 *  Created on: Jul 15, 2023
 *      Author: mushu
 */

#ifndef APP_COAP_HELPER_H_
#define APP_COAP_HELPER_H_


// Begin C scope
#ifdef __cplusplus
extern "C"
{
#endif

// Include libraries
/* Standard Library Header files */
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* OpenThread public API Header files */
#include <openthread/dataset.h>
#include <openthread/platform/logging.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>
#include <openthread/coap.h>
#include <openthread/ip6.h>
#include <openthread/ping_sender.h>
#include <openthread/link.h>

// Constants

// Data types
typedef struct CoapContext {
    void(* responseHandler)(otMessage *aMessage, const otMessageInfo *aMessageInfo, otError aResult);
} CoapContext;


// Public prototypes
void coapRequest(const otIp6Address* serverAddress, uint16_t serverPort, const char* uri, const char* payload);
void coapGetRequestWithContext(CoapContext* ctx, const otIp6Address* serverAddress, uint16_t serverPort, const char* uri);
void coapGetRequest(const otIp6Address* serverAddress, uint16_t serverPort, const char* uri);
uint16_t coapReadMessage(otMessage *aMessage, uint8_t* buff, uint16_t size);

// End of C scope
#ifdef __cplusplus
}
#endif


#endif /* APP_COAP_HELPER_H_ */
