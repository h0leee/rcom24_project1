#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_


#define BAUDRATE 38400
#define FLAG 0x7E
#define ESC 0x7D
#define A_ER 0x03 // comandos transmissor, respostas recetor
#define A_RE 0x01 // comandos recetor, respostas transmissor 
#define C_SET 0x03
#define C_DISC 0x0B
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55



typedef enum {
    START, 
    A, 
    C, 
    F, 
    STOP, 
    BCC1, 
    ESC_FOUND,
    READING_PAYLOAD
} LinkLayerState;


LinkLayerState stateMachine(int frameType);

int sendSupervisionFrame(int fd, unsigned char A_byte, unsigned char C_byte);

void displayStatistics();

unsigned char getControlFrame(int fd);

int byteStuffing(const unsigned char* inputMsg, int inputSize, unsigned char* outputMessage);

#endif 