/**/
#ifndef __HISIGNALLING_H_
#define __HISIGNALLING_H_
#include <stdio.h>
//#include "cnn_gender_classify.h"
#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */


#define HISIGNALLING_MSG_HEADER_LEN         (1)
#define HISGNALLING_MSG_FRAME_HEADER_LEN    (2)
#define HISIGNALLING_MSG_HEADER_TAIL_LEN    (3)
#define HISGNALLING_FREE_TASK_TIME          (10)
#define HISIGNALLING_MSG_MOTOR_ENGINE_LEN	(11)
#define HISIGNALLING_MSG_ONE_FRAME_LEN		(16)
#define HISIGNALLING_MSG_BUFF_LEN           (512)

typedef struct hisignalling_protocal {
    unsigned char frame_header[HISGNALLING_MSG_FRAME_HEADER_LEN];
    unsigned char hisignalling_msg_buf[HISIGNALLING_MSG_BUFF_LEN];
    unsigned int hisigalling_msg_len;
    unsigned char end_of_frame;
    unsigned int hisignalling_crc32_check; 
}hisignalling_protocal_type;

typedef enum hisignalling_return_val{
    HISIGNALLING_RET_VAL_CORRECT = 0,
    HISIGNALLING_RET_VAL_ERROR,
    HISGNALLING_RET_VAL_MAX
}hisignalling_error_type;

typedef enum{
    FIST = 0x01,
    PALM = 0x02,
    OTHERS = 0x03,
}HandClassification;

/*
*hisignalling protocal Function declaration
*/
unsigned int hisignalling_msg_task(void);
int usbOpenInit(void);
int uartOpenInit(void);
void usbUartSendRead(int fd, HandClassification refuseType);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __NNIE_SAMPLE_PLUG_H */

