#include <stdint.h>

void toggleLogger(void);
void notifyToggleLogger(void);
void rtos_log_task(void);
void init_log_task(void);
void logRaw(const uint8_t* data, uint32_t len);

typedef enum
{
  RTOS,
  USB,
  CAN,
  LIN,
  ETH,
  SENT,
  CXPI,
  LOGC_MAX
} logc;
