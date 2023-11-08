#include <pb_encode.h>
#include <returnQueue.h>

extern QueueHandle_t usbTxQueueHandle;

static uint32_t dataLostCntr = 0;

bool queueMessage(struct QueueMsgObject* msg, const pb_msgdesc_t fields[], const void *src_struct )
{
  /* Create and encode an Usb Command Object */
  pb_ostream_t ostream = pb_ostream_from_buffer(msg->uco.data, sizeof(msg->uco.data));
  bool status = pb_encode(&ostream, fields, src_struct);

  if(status)
  {
    msg->uco.header.payloadlength = (uint16_t)ostream.bytes_written;
    if(pdTRUE != xQueueSendToBack(usbTxQueueHandle, &msg, portMAX_DELAY))
    {
      /* If the pointer could not be queued to the usbTxQueue, return it to the original queue */
      xQueueSendToFront(msg->returnQueue, &msg, 0);
      dataLostCntr++;
    }
  }
  else
  {
    /* It is very unlikely encoding fails, but in case return the UCO to the original queue */
    xQueueSendToFront(msg->returnQueue, &msg, 0);
  }
  
  return status;
}

bool queueMessageFromISR(struct QueueMsgObject* msg, const pb_msgdesc_t fields[], const void *src_struct)
{
  /* Create and encode an Usb Command Object */
  pb_ostream_t ostream = pb_ostream_from_buffer(msg->uco.data, sizeof(msg->uco.data));
  bool status = pb_encode(&ostream, fields, src_struct);
  
  if(status)
  {
    msg->uco.header.payloadlength = (uint16_t)ostream.bytes_written;
    if(pdTRUE != xQueueSendToBackFromISR(usbTxQueueHandle, &msg, 0))
    {
      /* If the pointer could not be queued to the usbTxQueue, return it to the original queue */
      xQueueSendToFrontFromISR(msg->returnQueue, &msg, 0);
      dataLostCntr++;
    }
  }
  else
  {
    /* It is very unlikely encoding fails, but in case return the UCO to the original queue */
    xQueueSendToFrontFromISR(msg->returnQueue, &msg, 0);
  }
  
  return status;
}