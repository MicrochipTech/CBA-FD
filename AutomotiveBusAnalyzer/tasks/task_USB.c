#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <assert.h>
#include <USBDescriptors.h>
#include <USBRequests.h>
#include <USBD.h>
#include <USBDDriver.h>

#include "config.h"
#include "rtos.h"
#include "compiler.h"
#include "USB_Commands.h"
#include "returnQueue.h"
#include "ABAControl.h"
#include "tasks/task_USB.h"
#include "tasks/task_CAN.h"

/** DEBUG Start **/
// #define DISABLE_USBTX
/** DEBUG End **/

static volatile uint32_t usbLocked = 0;
static volatile uint32_t objectsTransfered = 0;
static volatile uint8_t usbConnected = 0;

/** USB standard device driver. */
static USBDDriver usbdDriver;
extern const USBDDriverDescriptors ABADriverDescriptors;

#define USB_TX_BUFFERS  2u
#define USB_INIT_DONE   0x1u
#define USB_TX_DONE     0x2u

//COMPILER_ALIGNED(32) COMPILER_SECTION(".data_TCM") uint8_t gTxBuffer[USB_TX_BUFFERS][USB_TX_SIZE];
COMPILER_ALIGNED(32) uint8_t gTxBuffer[USB_TX_BUFFERS][USB_TX_SIZE];
COMPILER_ALIGNED(32) COMPILER_SECTION(".data_TCM") static uint8_t gRxBuffer[USB_RX_SIZE];

/* USB TX QUEUE */
/* Only describes the Tx buffer queue. Processed buffers are put back into their origin queue
 * specified by the returnQueue pointer in the QueueMsgObject.
 */
#define USB_TX_QUEUE_DEPTH 32u
static uint8_t usbQueueListBuffer[USB_TX_QUEUE_DEPTH * sizeof(struct QueueMsgObject*)];
static StaticQueue_t xStaticQueueUsbTx;
QueueHandle_t usbTxQueueHandle;

/*
 * Calling undefined endpoints will cause an assertion
 */
static int32_t uco_DummyHandler(struct USBCMDObject* uco)
{
  assert(0);
  return 0;
}

tUcoHandler ucoHandlers[UCO_TYPE_MAX] = 
{
  uco_ApiHandler,
  uco_CanHandler,
  uco_DummyHandler,
  uco_DummyHandler,
  uco_DummyHandler
};

void init_usb_task(void)
{
  memset(gRxBuffer, 0, sizeof(gRxBuffer));
  memset(gTxBuffer, 0, sizeof(gTxBuffer));
  memset(&usbdDriver, 0, sizeof(USBDDriver));
  USBDDriver_Initialize(&usbdDriver, &ABADriverDescriptors, 0);

  usbTxQueueHandle = xQueueCreateStatic(USB_TX_QUEUE_DEPTH, sizeof(struct QueueMsgObject*), usbQueueListBuffer, &xStaticQueueUsbTx);
}

/**
 * USB Rx Callback
 */
COMPILER_SECTION(".code_TCM")
void _UsbDataReceived(void* userData, uint8_t status, uint32_t received, uint32_t remaining)
{
  if(status == USBD_STATUS_SUCCESS)
  {
    Board_SetLed(LED_USB, LED_TRI_B, LED_PRIO_NONE);
    /* If data has been received notify the USB Rx Task */
    xTaskNotifyFromISR(hUsbRxTask, received, eSetValueWithOverwrite, NULL);
  }
  else
  {
    /* If there was an error while receiving data, request another read without notifying the task */
    USBD_Read(ABADriverDescriptors_BULKOUT, gRxBuffer, USB_RX_SIZE, (TransferCallback)_UsbDataReceived, 0);
  }
}

/**
 * USB Tx Callback
 */
COMPILER_SECTION(".code_TCM")
void _UsbDataSent(void* userData, uint8_t status, uint32_t transferred, uint32_t remaining)
{ 
  if(status != USBD_STATUS_SUCCESS)
  {
    (void)userData;
    Board_SetLed(LED_USB, LED_TRI_R, LED_PRIO_HIGH);
  }

  Board_SetLed(LED_USB, LED_TRI_B, LED_PRIO_NONE);
  xTaskNotifyFromISR(hUsbTask, USB_TX_DONE, eSetBits, NULL);
}

COMPILER_SECTION(".code_TCM")
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
  USBDDriver_RequestHandler(&usbdDriver, request);
}

void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
  /* Notify the USB task the stack has been configured */
  if(cfgnum == 1u)
  {
    usbConnected = 1;
    Board_SetLed(LED_USB, LED_TRI_G, LED_PRIO_CLEAR);
    xTaskNotifyFromISR(hUsbTask, USB_INIT_DONE, eSetBits, NULL);
    USBD_Read(ABADriverDescriptors_BULKOUT, gRxBuffer, USB_RX_SIZE, (TransferCallback)_UsbDataReceived, 0);
  }
}

void USBDCallbacks_Reset(void)
{
  usbConnected = 0;
  Board_SetLed(LED_USB, LED_TRI_OFF, LED_PRIO_CLEAR);
}

void USBDCallbacks_Initialized(void)
{
}

/**
 * FreeRTOS USB Rx Task
 * The Rx task takes all USB Command Objects from the queue and passes them to the according handler
 */
void rtos_usb_task_rx()
{
  uint32_t received = 0;

  while(1)
  {
    xTaskNotifyWait(0, UINT32_MAX, &received, portMAX_DELAY);
    
    uint32_t bytesLeft = received;
    uint8_t* rdPtr = &gRxBuffer[0];

    while(bytesLeft >= sizeof(struct USBCMDHeader))
    {
      struct USBCMDObject* uco = (struct USBCMDObject*)rdPtr;
      uint32_t length = sizeof(struct USBCMDHeader) + uco->header.payloadlength;
      if(bytesLeft >= length)
      {
        if(uco->header.type < UCO_TYPE_MAX)
        {
          ucoHandlers[uco->header.type](uco);
        }
        rdPtr += length;
        bytesLeft -= length;
      }
      else
      {
        bytesLeft = 0;
      }
    }

    Board_SetLed(LED_USB, LED_TRI_G, LED_PRIO_CLEAR);
    USBD_Read(ABADriverDescriptors_BULKOUT, gRxBuffer, USB_RX_SIZE, (TransferCallback)_UsbDataReceived, 0);
  }
}

/**
 * FreeRTOS USB Tx task
 * The Tx task copies the USB Command Objects from the queue into the Tx buffer and triggers the DMA 
 */

COMPILER_SECTION(".code_TCM")
void rtos_usb_task()
{
  uint32_t notificationVal = 0;
  USBD_Init();
  vTaskDelay(10u);
  USBD_Connect();

  /* Wait for the USB stack to be configured */
  xTaskNotifyWait(0, 0, 0, portMAX_DELAY);
  xTaskNotifyFromISR(hUsbTask, USB_TX_DONE, eSetBits, NULL);

  uint8_t bufferIdx = 0;
  uint8_t* pTxBuffer = gTxBuffer[0];
  uint32_t sendLen = 0;
  uint32_t spaceLeft = USB_TX_SIZE;

  while(1)
  {
    struct QueueMsgObject* qObject = NULL;
    
    /* Grab a Tx object from the queue */
    if(pdTRUE == xQueueReceive(usbTxQueueHandle, &qObject, 10))
    {
      assert(qObject != 0);
      
      /* Check if object will fit into the Tx buffer */
      uint32_t length = sizeof(qObject->uco.header) + qObject->uco.header.payloadlength;
      if(spaceLeft > length)
      {
        /* Tx buffer still has some space left */
        memcpy(pTxBuffer, &qObject->uco, length);
        sendLen += length;
        pTxBuffer += length;
        spaceLeft -= length;

        /* Return the queue element */
        xQueueSendToBack(qObject->returnQueue, &qObject, 0);
        objectsTransfered++;
      }
      else
      {
        /* Re-queue the queue element so it will be put into the next Tx buffer */
        xQueueSendToFront(usbTxQueueHandle, &qObject, 0);
        spaceLeft = 0;
      }
    }
    else
    {
      /* No new elements in usbTxQueue */
      /* Set spaceLeft to 0 which triggers a transfer */
      spaceLeft = 0;
    }

    if(usbConnected)
    {
      /* There is no more additional space in the TX buffer so transmit it */
      if(spaceLeft == 0)
      {
#ifndef DISABLE_USBTX
        if(sendLen == 0)
        {
          spaceLeft = USB_TX_SIZE;
          continue;
        }
        
        /* Wait until the USB Tx transfer has finished */
        do {
          xTaskNotifyWait(0, 0xFFu, &notificationVal, portMAX_DELAY);
        } while ((notificationVal & USB_TX_DONE) == 0);

        /* Invoke DMA */
        Board_SetLed(LED_USB, LED_TRI_G, LED_PRIO_NONE);
        if(USBRC_SUCCESS != USBD_Write(ABADriverDescriptors_BULKIN, gTxBuffer[bufferIdx], sendLen, _UsbDataSent, 0))
        {
          Board_SetLed(LED_USB, LED_TRI_R, LED_PRIO_HIGH);
        }
#endif
        /* Switch to other Tx buffer, so it can be filled while USB DMA is still active */
        bufferIdx = (bufferIdx + 1) % USB_TX_BUFFERS;
        spaceLeft = USB_TX_SIZE;
        pTxBuffer = gTxBuffer[bufferIdx];
        sendLen = 0;
      }
    }
  }
}
