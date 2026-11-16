#ifndef IAP_H
#define IAP_H

#include <stdint.h>

#define APP_ADDRESS 0x08004000U
#define APP_MAX_SIZE (44UL * 1024UL)

int iap_copy_firmware(uint32_t fw_size);
void jump_to_app(void);

#endif