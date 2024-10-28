#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include "link_layer.h"

#define BAUDRATE 38400
#define FLAG 0x7E
#define ESC 0x7D
#define A_ER 0x03
#define A_RE 0x01
#define C_SET 0x03
#define C_DISC 0x0B
#define C_UA 0x07
#define C_RR(Nr) ((Nr << 7) | 0x05)
#define C_REJ(Nr) ((Nr << 7) | 0x01)
#define C_N(Ns) (Ns << 6)

typedef enum {
    START, A, C, F, STOP, BCC1, BCC2, READING_PAYLOAD
} LinkLayerState;


LinkLayerState stateMachines(LinkLayer* connectionParameter, int frameType);

#endif 