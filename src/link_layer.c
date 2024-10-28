// Link layer protocol implementation


// depois poderei tentar modular o código da stateMachine 

#include "link_layer.h"
#include "serial_port.h"

#define A_RE = 0x01


volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
int timeOut = 0;
int retransmissions = 0;
unsigned char tramaTx = 0;
unsigned char tramaRx = 1;





typedef enum {
    START, A, C, F, STOP, BCC1, BCC2, ESC, READING_PAYLOAD
} LinkLayerState;


LinkLayerState stateMachines(LinkLayer connectionParameter, int frameType) {

}




// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}


int connection(const char *serialPort) {

    int fd = connection(serialPort);
    if (fd < 0) {
        perror(serialPort);
        return -1; 
    }


    struct termios oldtio;
    struct termios newtio;

    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    return fd;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
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
    printf("New termio structure set\n");

    int alarmTriggered = FALSE;
    LinkLayerState machineState = START;
    sleep(1); // O sleep espera até que todos os bytes tenham sido escritos
    unsigned char byteRead;
    int timeOut = connectionParameters.timeout;
    int retransmissions = connectionParameters.nRetransmissions;

    switch (connectionParameters.role) {
        case LlTx: {
                // Quando o sinal SIGALRM for acionado, a função alarmHandler será chamada.
                (void) signal(SIGALRM, alarmHandler); 

                while (machineState != STOP && !alarmTriggered) {

                    if(read(fd, &byteRead, 1) > 0) {
                        switch(machineState) {
                            case START:
                                if (byteRead == FLAG) machineState = F;
                                break;
                            
                            case F:
                                if(byteRead == 0x01) machineState = A;
                                break;

                            case A:
                                if(byteRead == C_RE) machineState = C;
                                break;

                            case C:
                                if(byteRead == BCC1) machineState = BCC1;
                                break;

                            case BCC1:
                                if(byteRead == FLAG) machineState = STO;
                                break;

                            default:
                                break;
                        }
                    }

                }
                break; // Adicionado break para evitar fall-through
        }

        
        case LlRx: {
            // Implementação para LlRx
            break; // Adicionado break para evitar fall-through
        }
    }

    return 0;
}





/////////////////////////////////////////////////////
//STUFFING
/////////////////////////////////////////////////////

//Stuffing e o processo inverso vao ser usados no LLWRITE

int byteStuffing(const unsigned char *inputMsg, int inputSize, unsigned char *outputMsg)
{
    int stuffedSize = 0; //Variável para guardar o tamanho da mensagem (quantos bytes foram escritos em outputMsg)

    outputMsg[stuffedSize++] = inputMsg[0]; //o stuffedSize serve como indice para avançar para a próxima posição

    printf("\nSTUFFING STARTED\n"); //debug

    printf("%x\n", outputMsg[stuffedSize - 1]); 

    for (int i = 1; i < inputSize; i++)
    {
        if (inputMsg[i] == FLAG || inputMsg[i] == ESCAPE) //bytes de controlo, se for igual é necessário stuffing
        {
            outputMsg[stuffedSize++] = ESCAPE; //indica que o próximo byte foi modificado
            outputMsg[stuffedSize++] = inputMsg[i] ^ 0x20; 

        }
        else
        {
            //se não for FLAG ou ESCAPE vai copiar o byte logo
            outputMsg[stuffedSize++] = inputMsg[i];

        }
    }

    printf("\nSTUFFING COMPLETED\n"); //debug

    return stuffedSize;
}


// função usada para reverter o byte stuffing aplicado na mensagem

int byteDestuffing(const unsigned char *stuffedMsg, int stuffedSize, unsigned char *originalMsg)
{
    int destuffedSize = 0; // vai contar quantos bytes sao escritos

    printf("\nDESTUFFING STARTED\n"); // debug

    originalMsg[destuffedSize++] = stuffedMsg[0]; 

    for (int i = 1; i < stuffedSize; i++)
    {
        if (stuffedMsg[i] == ESCAPE) // vemos se o byte foi mudificado com stuffing para saber se temos de restaurar
        {
            originalMsg[destuffedSize++] = stuffedMsg[i + 1] ^ 0x20; // byte é restaurado e incrementamos para a posição seguinte
            i++;
        }
        else
        {
            originalMsg[destuffedSize++] = stuffedMsg[i]; // se nao for ESCAPE copia logo
        }
    }

    printf("\nDESTUFFING COMPLETED\n"); // debug

    return destuffedSize;
}


// BCC -> block check character

unsigned char computeBCC2(const unsigned char *buffer, int length, int startByte)
{
    if (length < 0) // se o tamanho do buffer for negativo como é obvio retornará erro mas sem interromper a execução
    {
        printf("Error: Buffer size is %d\n", length);
    }
    unsigned char BCC2Value = 0x00; // inicializamos a zero para fazer o XOR

    for (unsigned int i = startByte; i < length; i++) // este loop vaipercorrer o buffer
    {
        BCC2Value ^= buffer[i]; // ^= é o XOR que é aplicado ao BCC2
    }
    return BCC2Value;
}







////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int llwrite(const unsigned char *buf, int bufSize)
{
    int totalSize = bufSize + 5; // tamanho total do frame

    unsigned char frame[totalSize]; // variavel para armazenar o frame antes do stuffing

    static int frameSequence = 0; 

    frame[0] = FLAG;
    frame[1] = A_T; // byte de endereço
    frame[2] = C_INF(frameSequence); // byte de controlo
    frame[3] = BCC(A_T, C_INF(frameSequence)); // BCC calculado entre o A e o C

    unsigned char calculatedBCC2 = buf[0]; 

    for (int i = 0; i < bufSize; i++)
    {
        frame[i + 4] = buf[i];
        if (i > 0)
            calculatedBCC2 ^= buf[i]; // é usado o XOR novamente para calcular o BCC2
    }

    frame[bufSize + 4] = calculatedBCC2; // é inserido no final do frame

    unsigned char stuffedFrame[totalSize * 2]; // vai armazenar o frame depois do stuffing

    // aqui vamos aplicar o stuffing ao frame e copiamos para o stuffedframe
    totalSize = byteStuffing(frame, totalSize, stuffedFrame); 
    stuffedFrame[totalSize] = FLAG; // a flag assinala o fim do processo
    totalSize++;


    //inicisalização das variaveis para controlar o alarme e state machine
    STOP = FALSE; 
    alarmEnabled = FALSE;
    alarmCount = 0;
    state = START;

    // aqui contamos o nr de tentativas e temos rejected para ver se o frame é rejeitado ou nao
    int attemptCount = 0;
    int rejected = FALSE;


    while (!STOP && alarmCount < layer.nRetransmissions) // while até o stop ser true ou seja ate o frame ser aceite
    {

        // se o alarmenabled for false vai fazer uma tentativa de envio
        if (!alarmEnabled) 
        {
            attemptCount++;
            bytes = write(fd, stuffedFrame, totalSize); // aqui enviamos ao usar o write
            alarm(layer.timeout);
            alarmEnabled = TRUE;
            state = START; // start para iniciar a state machine
        }

        stateMachine(A_T, NULL, 0, 1); // chamamos a state machine de cima

        if ((frameSequence == 0 && response == REJ1) || (frameSequence == 1 && response == REJ0)) // verificamos se o frame foi ou nao rejeitado, se REJ1 entao rejected = true
        {
            rejected = TRUE;
        }

        if (rejected) // se foi rejeitado desativa o alarme para tentar enviar outra vez
        {
            alarm(0);
            alarmEnabled = FALSE;
        }
    }

    frameSequence = (frameSequence + 1) % 2;

    alarm(0); //alarme é desligado depois de ter conseguido enviar

    printf("Data Successfully Accepted!\n"); //debug

    return 0;
}








////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *buffer)
{
    int bytesread = 0; // vai contar o numero de bytes lidos do frame recebido

    static int packet = 0; 

    unsigned char stuffedMsg[MAX_BUFFER_SIZE]; // vai armazenar o frame com byte stuffing
    unsigned char unstuffedMsg[MAX_PACKET_SIZE + 7]; // vai armazenar o frame depouis do byte destuffing

    STOP = FALSE;
    state = START;

    int bytes = 0; // vai contar o numero de bytes de cada leitura

    while (STOP == FALSE) // neste loop vai ler bytes ate receber tudo
    {
        if (stateMachine(A_T, C_INF(packet), 1, 0)) // a state machine vai processar os daddos recebidos
        {
            stuffedMsg[bytesread] = readbyte;
            bytesread++;
        }
    }

    printf("DATA RECEIVED\n"); // debug

    int s = destuffing(stuffedMsg, bytesread, unstuffedMsg); // destuffing e guardamos o tamanho total da mensagem em s

    unsigned char receivedBCC2 = unstuffedMsg[s - 2]; 

    unsigned char expectedBCC2 = calculateBCC2(unstuffedMsg, s - 2, 4);

    if (receivedBCC2 == expectedBCC2 && unstuffedMsg[2] == C_INF(packet)) // verificamos se o recebido era o que era esperado e se o bytre de controlo esta correto 
    {
        packet = (packet + 1) % 2;
        sendBuffer(A_T, RR(packet)); //RR é uma mensagem para confirmar que o frame foi recebido
        memcpy(buffer, &unstuffedMsg[4], s - 5); // é feita uma copia para o buffer
        return s - 5;
    }

    //este else usamos para descartar frame duplicado mas confirmamos na mesma a receção com o RR
    else if (receivedBCC2 == expectedBCC2)
    {
        sendBuffer(A_T, RR(packet));
        tcflush(fd, TCIFLUSH); // aqui limpamos o buffer
        printf("Duplicate packet!\n");
    }
    // se o BCC2 recebido nao for o esperado é enviada o REJ (mensagem de rejeiçãop)
    else
    {
        sendBuffer(A_T, REJ(packet));
        tcflush(fd, TCIFLUSH); //limpamos na mesma o buffer
        printf("Error in BCC2, sent REJ\n");
    }
    return -1;
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
