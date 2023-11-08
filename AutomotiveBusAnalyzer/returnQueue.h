/*
 * returnQueue.h
 *
 * Created: 20.02.2018 07:32:40
 *  Author: M43734
 */ 


#ifndef RETURNQUEUE_H_
#define RETURNQUEUE_H_

#include "compiler.h"
#include <USB_Commands.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <pb.h>

struct QueueMsgObject
{
  struct USBCMDObject uco;
  QueueHandle_t returnQueue;
};

bool queueMessage(struct QueueMsgObject* msg, const pb_msgdesc_t fields[], const void *src_struct);
bool queueMessageFromISR(struct QueueMsgObject* msg, const pb_msgdesc_t fields[], const void *src_struct);


#endif /* RETURNQUEUE_H_ */
