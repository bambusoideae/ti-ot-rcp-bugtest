/*
 * coap_helper.c
 *
 *  Created on: Jul 15, 2023
 *      Author: mushu
 */


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

/* OpenThread Internal/Example Header files */
#include "otsupport/otrtosapi.h"
#include "otsupport/otinstance.h"

//
#include "utils/code_utils.h"
#include "disp_utils.h"
#include "otstack.h"

// Self header
#include "coap_helper.h"

// Constants
// Default COAP Configuration
#define DEFAULT_COAP_HEADER_TOKEN_LEN 2


// Declare functions
// Response handler
static void coapResponseHandler(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo, otError aResult) {
    // if (aResult != OT_ERROR_NONE) {
    //     return;
    // } else if ((aMessageInfo != NULL) && (aMessage != NULL)) {
    // }
    CoapContext* ctx = (CoapContext*) aContext;
    ctx->responseHandler(aMessage, aMessageInfo, aResult);
}

// Read message
uint16_t coapReadMessage(otMessage *aMessage, uint8_t* buff, uint16_t size) {
    uint16_t offset = otMessageGetOffset(aMessage);
    uint16_t bodyLength = otMessageGetLength(aMessage) - offset;
    uint16_t nRead = otMessageRead(aMessage, offset, buff, size);
    buff[nRead] = '\0'; // C string terminator
    return nRead;
}

// POST Request
void coapRequest(const otIp6Address* serverAddress, uint16_t serverPort, const char* uri, const char* payload) {
    otError error = OT_ERROR_NONE;
    otMessage *requestMessage = NULL;
    otMessageInfo messageInfo;
    otInstance *instance = OtInstance_get();

    //
    OtRtosApi_lock();
    requestMessage = otCoapNewMessage(instance, NULL);
    OtRtosApi_unlock();
    otEXPECT_ACTION(requestMessage != NULL, error = OT_ERROR_NO_BUFS);

    //
    OtRtosApi_lock();
    otCoapMessageInit(requestMessage, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);
    otCoapMessageGenerateToken(requestMessage, DEFAULT_COAP_HEADER_TOKEN_LEN);
    error = otCoapMessageAppendUriPathOptions(requestMessage, uri);
    OtRtosApi_unlock();
    otEXPECT(OT_ERROR_NONE == error);

    //
    OtRtosApi_lock();
    otCoapMessageSetPayloadMarker(requestMessage);
    OtRtosApi_unlock();

    //
    OtRtosApi_lock();
    error = otMessageAppend(requestMessage, payload,
                            strlen((const char*) payload));
    OtRtosApi_unlock();
    otEXPECT(OT_ERROR_NONE == error);

    // Set Server info
    memset(&messageInfo, 0, sizeof(messageInfo));
    messageInfo.mPeerAddr = *serverAddress;
    messageInfo.mPeerPort = serverPort;

    // Send COAP Request
    OtRtosApi_lock();
    error = otCoapSendRequest(instance, requestMessage, &messageInfo, NULL,
                              NULL);
    OtRtosApi_unlock();

    // otExpect exit label
    exit:
    if (error != OT_ERROR_NONE && requestMessage != NULL)
    {
        OtRtosApi_lock();
        otMessageFree(requestMessage);
        OtRtosApi_unlock();
    }
}

// Declare functions
void coapGetRequestWithContext(CoapContext* ctx, const otIp6Address* serverAddress, uint16_t serverPort, const char* uri) {
    otError error = OT_ERROR_NONE;
    otMessage *requestMessage = NULL;
    otMessageInfo messageInfo;
    otInstance *instance = OtInstance_get();

    //
    OtRtosApi_lock();
    requestMessage = otCoapNewMessage(instance, NULL);
    OtRtosApi_unlock();
    otEXPECT_ACTION(requestMessage != NULL, error = OT_ERROR_NO_BUFS);

    //
    OtRtosApi_lock();
    if (ctx != NULL) {
        otCoapMessageInit(requestMessage, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_GET);
    } else {
        otCoapMessageInit(requestMessage, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_GET);
    }
    otCoapMessageGenerateToken(requestMessage, DEFAULT_COAP_HEADER_TOKEN_LEN);
    error = otCoapMessageAppendUriPathOptions(requestMessage, uri);
    OtRtosApi_unlock();
    otEXPECT(OT_ERROR_NONE == error);

    // Add marker
    OtRtosApi_lock();
    otCoapMessageSetPayloadMarker(requestMessage);
    OtRtosApi_unlock();

    // Set Server info
    memset(&messageInfo, 0, sizeof(messageInfo));
    messageInfo.mPeerAddr = *serverAddress;
    messageInfo.mPeerPort = serverPort;

    // Send COAP Request
    OtRtosApi_lock();
    if (ctx != NULL) {
        error = otCoapSendRequest(instance, requestMessage, &messageInfo, *coapResponseHandler,
                                          ctx);
    } else {
        error = otCoapSendRequest(instance, requestMessage, &messageInfo, NULL,
                                  NULL);
    }
    OtRtosApi_unlock();

    // otExpect exit label
    exit:
    if (error != OT_ERROR_NONE && requestMessage != NULL)
    {
        OtRtosApi_lock();
        otMessageFree(requestMessage);
        OtRtosApi_unlock();
    }
}

void coapGetRequest(const otIp6Address* serverAddress, uint16_t serverPort, const char* uri) {
    coapGetRequestWithContext(NULL, serverAddress, serverPort, uri);
}
