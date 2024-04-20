#ifndef __SUBARU_LEVORG_VNX_H__
#define __SUBARU_LEVORG_VNX_H__

/* #define DEBUG_MODE */

// Receive Only Two CAN Ids
#define CAN_ID_CCU 0x6BB
#define CAN_ID_SCU 0x32B
#define CAN_ID_TCU 0x048
#define CAN_ID_MCU 0x139

// CCU and TCU STATUS
enum cu_status {
    ENGINE_STOP,
    NOT_READY,
    READY,
    AVH_ON,
    AVH_OFF
};

// STATUS
enum status {
    PROCESSING,
    CANCELLED,
    FAILED,
    SUCCEEDED
};

// MODE
enum debug_mode {
    NORMAL,
    DEBUG,
    CANDUMP
};

extern enum debug_mode DebugMode;

// for Calculate Check Sum
#define SUM_CHECK_ADDER (-0x3F)


#define MAX_RETRY 2

#endif /* __SUBARU_LEVORG_VNX_H_ */
