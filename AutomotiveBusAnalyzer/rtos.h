/*
 * rtos.h
 *
 * Created: 20.02.2018 13:10:24
 *  Author: M43734
 */ 


#ifndef RTOS_H_
#define RTOS_H_

extern uint32_t init_task_callback(void);
extern void rtos_main_task(void);
extern void rtos_can_task(void);
extern void rtos_usb_task(void);
extern void rtos_eth_task(void);
extern void rtos_usb_task_rx(void);
extern void TimeTick_Tick(void);
extern void rtos_run(void);

extern TaskHandle_t hCanTask;
extern TaskHandle_t hUsbTask;
extern TaskHandle_t hUsbRxTask;
extern TaskHandle_t hLogTask;

#endif /* RTOS_H_ */