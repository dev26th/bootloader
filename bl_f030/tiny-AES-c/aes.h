// based on https://github.com/kokke/tiny-AES-c, commit 268d40d97c9866778e0ce6b844b26b288c6f0909

#ifndef _AES_H_
#define _AES_H_

#include <stdint.h>

void AES_CBC_init(const uint8_t* key, const uint8_t* iv);
void AES_CBC_decrypt_buffer(uint8_t* buf, uint32_t length);

#endif //_AES_H_
