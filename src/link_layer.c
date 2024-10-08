// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <unistd.h>
#include <signal.h>
#include <stdio.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

#define BAUDRATE B38400 //Velocidade de transmissão de dados (simbolos != bits)
#define BUF_SIZE 256

volatile int STOP = FALSE;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    // (Semelhante ao writenoncanonical.c)

    //Abrimos a porta de série para leitura e escrita. 

    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0){
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    //Estruturas usadas para configurar parâmetros da porta de série.

    struct termios oldtio; //oldtio armazena a configuração atual da porta de série.
    struct termios newtio; //newtio armazena a nova configuração da porta de série.

    //Guardar as definições da porta de série.

    if(tcgetattr(fd, &oldtio) == -1){
        perror("tcgetattr");
        exit(-1);
    }

    //Limpar a struct para armazenar as novas configurações da porta de série.

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR; //Ignora erros de paridade ao receber dados.
    newtio.c_oflag = 0; 

    newtio.c_lflag = 0; //Configura a comunicação em modo noncanonical
    newtio.c_cc[VTIME] = 0; 
    newtio.c_cc[VMIN] = 5;  //A função read só dá return depois de serem recebidos pelo menos 5 caracteres.

    




    // TODO

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
