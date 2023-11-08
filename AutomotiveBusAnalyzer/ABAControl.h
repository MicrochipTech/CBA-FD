/*
 * ABAControl.h
 *
 * Created: 23.02.2018 07:59:18
 *  Author: M43734
 */ 


#ifndef ABACONTROL_H_
#define ABACONTROL_H_

void init_aba_api(void);
int32_t uco_ApiHandler(struct USBCMDObject* uco);

#endif /* ABACONTROL_H_ */
