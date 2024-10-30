// Link layer protocol implementation

// depois poderei tentar modular o código da stateMachine
// llread, stateMachine, getControlPacket, displayStatistics

#include "link_layer.h"
#include "serial_port.h"
#include "constants.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

volatile int STOP = FALSE;
int alarmFired = FALSE;
int alarmCount = 0;
int numberRetransmissions = 0;
int timeOut = 0;
LinkLayerState machineState;
LinkLayerRole role;
int fd;
int txFrame = 0, rxFrame = 1;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmFired = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int makeConnection(LinkLayer *connectionParameters)
{

    fd = openSerialPort(connectionParameters->serialPort, connectionParameters->baudRate);

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

    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
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
    // Abrir porta serial e verificar erro
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    int fd = makeConnection(&connectionParameters);
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        return -1;
    }

    printf("New termio structure set\n");

    int alarmTriggered = FALSE;
    machineState = START;
    unsigned char byteRead;
    timeOut = connectionParameters.timeout;
    numberRetransmissions = connectionParameters.nRetransmissions; // Usar uma cópia local para preservar o valor original

    switch (connectionParameters.role)
    {
    case LlTx:

        (void)signal(SIGALRM, alarmHandler);

        int retransmissions = numberRetransmissions;

        while (retransmissions > 0 && machineState != STOP)
        {

            sendSupervisionFrame(fd, A_ER, C_SET);
            alarm(timeOut);
            alarmTriggered = FALSE;

            while (machineState != STOP && !alarmTriggered)
            {
                if (read(fd, &byteRead, 1) > 0)
                {
                    switch (machineState)
                    {
                    case START:
                        if (byteRead == FLAG)
                            machineState = F;
                        break;
                    case F:
                        if (byteRead == A_RE)
                            machineState = A;
                        break;
                    case A:
                        if (byteRead == C_UA)
                            machineState = C;
                        break;
                    case C:
                        if (byteRead == (A_RE ^ C_UA))
                            machineState = BCC1;
                        break;
                    case BCC1:
                        if (byteRead == FLAG)
                            machineState = STOP;
                        else
                            machineState = START;
                        break;
                    default:
                        break;
                    }
                }
            }

            retransmissions--; // Reduz o número de retransmissões
        }
        if (machineState != STOP)
            return -1;
        break;

    case LlRx:
        while (machineState != STOP)
        {
            if (read(fd, &byteRead, 1) > 0)
            { // Corrigido o parêntesis aqui
                switch (machineState)
                {
                case START:
                    if (byteRead == FLAG)
                        machineState = F;
                    break;
                case F:
                    if (byteRead == A_ER)
                        machineState = A;
                    break;
                case A:
                    if (byteRead == C_SET)
                        machineState = C;
                    break;
                case C:
                    if (byteRead == (A_ER ^ C_SET))
                        machineState = BCC1;
                    break;
                case BCC1:
                    if (byteRead == FLAG)
                        machineState = STOP;
                    else
                        machineState = START;
                    break;
                default:
                    break;
                }
            }
        }
        sendSupervisionFrame(fd, A_RE, C_UA); // Enviar quadro de supervisão de resposta
        break;

    default:
        return -1;
    }

    return fd; // Retorna o descritor de arquivo da conexão bem-sucedida
}

/////////////////////////////////////////////////////
// STUFFING
/////////////////////////////////////////////////////

// Stuffing e o processo inverso vao ser usados no LLWRITE

int byteStuffing(const unsigned char *inputMsg, int inputSize, unsigned char *outputMsg)
{
    int stuffedSize = 0; 

    printf("\nSTUFFING STARTED\n"); // só para debug

    printf("%x\n", outputMsg[stuffedSize - 1]);

    for (int i = 0; i < inputSize; i++)
    {
        if (inputMsg[i] == FLAG || inputMsg[i] == ESC) // bytes de controlo, se for igual é necessário stuffing
        {
            if(stuffedSize + 2 > MAX_PAYLOAD_SIZE) {
                printf("Buffer de saída excedido a fazer byte stuffing\n");
            }

            outputMsg[stuffedSize++] = ESC; 
            outputMsg[stuffedSize++] = inputMsg[i] ^ 0x20;
        }
        else {
            if(stuffedSize + 1 > MAX_PAYLOAD_SIZE) {
                printf("Buffer de saída excedido a fazer byte stuffing\n");
            }
            outputMsg[stuffedSize++] = inputMsg[i];
        }
    }

    // Opcional: Adicionar um terminador nulo se necessário
    // if (stuffedSize < maxOutputSize) {
    //     outputMsg[stuffedSize] = '\0';
    // }


    printf("\nSTUFFING FEITO COM SUCESSO\n"); // debug

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
        if (stuffedMsg[i] == ESC) // vemos se o byte foi mudificado com stuffing para saber se temos de restaurar
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
    int totalSize = bufSize + 6;    // tamanho total do frame, payload e 6 bytes adicionais
    unsigned char frame[totalSize]; // variável para armazenar o frame antes do stuffing

    frame[0] = FLAG;
    frame[1] = A_ER;                 // byte de endereço
    frame[2] = C_INF(txFrame); // byte de controlo
    frame[3] = frame[1] ^ frame[2];  // BCC calculado entre o A e o C

    unsigned char calculatedBCC2 = buf[0];
    for (int i = 1; i < bufSize; i++)
        calculatedBCC2 ^= buf[i];

    // Copiar o buffer de dados, a partir do index 4, para o frame
    memcpy(&frame[4], buf, bufSize);
    frame[bufSize + 4] = calculatedBCC2; // BCC2 inserido no final do frame

    unsigned char stuffedFrame[totalSize * 2]; // vai armazenar o frame depois do stuffing
    totalSize = byteStuffing(frame, totalSize, stuffedFrame);
    stuffedFrame[totalSize] = FLAG; // FLAG para o final do frame
    totalSize++;

    STOP = FALSE;
    alarmFired = FALSE;
    alarmCount = 0;
    int attemptCounter = 0;
    int failedTransmission = 0, successfulTransmission = 0;

    while (attemptCounter < numberRetransmissions) // Tenta até ao limite de retransmissões
    {
        alarmFired = FALSE;
        alarm(timeOut); // Define o temporizador

        failedTransmission = 0;
        successfulTransmission = 0;

        ssize_t bytesWritten = write(fd, stuffedFrame, totalSize);
        if (bytesWritten < 0)
        {
            perror("Erro ao escrever o frame");
            continue; // Tenta novamente
        }

        while (!failedTransmission && !successfulTransmission && !alarmFired)
        {
            int result = getControlFrame(fd);

            if (result == -1)
                continue;

            else if (result == C_REJ(0) || result == C_REJ(1))
                failedTransmission = 1;

            else if (result == C_RR(0) || result == C_RR(1))
            {
                successfulTransmission = 1;
                txFrame = (txFrame + 1) % 2; // Alterna o frameSequence entre 0 e 1
            }
        }

        if (successfulTransmission)
            break; // Se bem-sucedido, sai do loop
        attemptCounter++;
    }

    if (!successfulTransmission)
    { // Se todas as tentativas falharam
        llclose(fd);
        return -1;
    }

    printf("Data Successfully Accepted!\n"); // debug
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char byteRead, cByte;
    LinkLayerState machineState = START; // Estado inicial da máquina de estados
    int frameIndex = 0;

    while (machineState != STOP) { // Lê bytes até que o frame completo seja recebido
        if (read(fd, &byteRead, 1) > 0)
        {
            switch(machineState)
            {
                case START:
                    if (byteRead == FLAG)
                        machineState = F;
                    break;

                case F:
                    if (byteRead == A_ER)
                        machineState = A;
                    else if (byteRead != FLAG)
                        machineState = START;
                    // Se byteRead == FLAG, permanece no estado F (pode ser início de uma nova trama)
                    break;

                case A:
                    if (byteRead == C_N(0) || byteRead == C_N(1))
                    {
                        machineState = C;
                        cByte = byteRead;
                    }
                    else if(byteRead == FLAG)
                        machineState = F;
                    else if(byteRead == C_DISC) {
                        sendSupervisionFrame(fd, A_RE, C_DISC);
                        return 0; // Indica desconexão
                    }
                    else
                        machineState = START;
                    break;

                case C:
                    if (byteRead == (A_ER ^ cByte))
                        machineState = BCC1;
                    else if (byteRead == FLAG)
                        machineState = F;
                    else
                        machineState = START;
                    break;

                case BCC1:
                    if(byteRead == ESC)
                        machineState = ESC_FOUND;

                    else if(byteRead == FLAG) {
                        if(frameIndex <= 0){

                            // Nenhum dado recebido antes da FLAG, frame inválido
                            machineState = START;
                            break;
                        }

                        unsigned char bcc2 = packet[frameIndex-1];
                        frameIndex--;

                        unsigned char acc = 0;

                        // Calcula BCC2 a partir do payload 
                        for(int i = 0; i < frameIndex; i++)
                            acc ^= packet[i];

                        if(acc == bcc2){
                            machineState = STOP;
                            sendSupervisionFrame(fd, A_RE, C_RR(rxFrame));
                            rxFrame = (rxFrame + 1) % 2;
                            return frameIndex; // Retorna o número de bytes de dados recebidos
                        }
                        else{
                            printf("Erro: retransmissão necessária.\n");
                            sendSupervisionFrame(fd, A_RE, C_REJ(rxFrame));
                            return -1; // Indica erro na receção
                        }
                    }
                    else {
                        if(frameIndex < MAX_PAYLOAD_SIZE){
                            packet[frameIndex++] = byteRead;
                        }
                        else{
                            printf("Erro: buffer de pacote excedido.\n");
                            machineState = START; // Reinicia a máquina de estados
                        }
                    }
                    break;

                case ESC_FOUND:
                    machineState = BCC1; // para continuar a ler payload 

                    if (byteRead == 0x5E) { // Representa FLAG original
                        packet[frameIndex++] = FLAG;
                    }
                    else if (byteRead == 0x5D) { // Representa ESC original
                        packet[frameIndex++] = ESC;
                    }
                    else{
                        printf("Erro: sequência de escape inválida.\n");
                        machineState = START; 
                    }
                    break;

                default: 
                    machineState = START; // não se conhece o estado, reinicia 
                    break;
            }
        }
    }
    return -1; // erro caso o loop termine sem receber STOP
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    machineState = START;
    unsigned char byteRead;
    (void)signal(SIGALRM, alarmHandler);

    int retransmissions = numberRetransmissions;

    switch (role)
    {
    case LlTx:
        // Transmissor: inicia desconexão
        while (retransmissions > 0 && machineState != STOP)
        {
            sendSupervisionFrame(fd, A_ER, C_DISC); // Envia DISC
            alarm(timeOut);
            alarmFired = FALSE;

            machineState = START;
            while (!alarmFired && machineState != STOP)
            {
                if (read(fd, &byteRead, 1) > 0)
                {
                    switch (machineState)
                    {
                    case START:
                        if (byteRead == FLAG)
                            machineState = F;
                        break;

                    case F:
                        if (byteRead == A_ER)
                            machineState = A;
                        else if (byteRead != FLAG)
                            machineState = START;
                        break;

                    case A:
                        if (byteRead == C_DISC)
                            machineState = C;
                        else if (byteRead == FLAG)
                            machineState = F;
                        break;

                    case C:
                        if (byteRead == (A_ER ^ C_DISC))
                            machineState = BCC1;
                        else if (byteRead == FLAG)
                            machineState = F;
                        break;

                    case BCC1:
                        if (byteRead == FLAG)
                            machineState = STOP;
                        else
                            machineState = START;
                        break;

                    default:
                        break;
                    }
                }
            }
            retransmissions--;
        }

        if (machineState != STOP)
            return -1;

        // Transmissor envia UA para acabar de vez comunicação
        sendSupervisionFrame(fd, A_ER, C_UA);
        break;

    case LlRx:

        // Recetor: aguarda DISC do transmissor
        while (machineState != STOP && retransmissions > 0)
        {
            alarm(timeOut);
            alarmFired = FALSE;

            machineState = START;

            while (!alarmFired && machineState != STOP)
            {
                if (read(fd, &byteRead, 1) > 0)
                {
                    switch (machineState)
                    {
                    case START:
                        if (byteRead == FLAG)
                            machineState = F;
                        break;

                    case F:
                        // esta cena não sei
                        if (byteRead == A_ER)
                            machineState = A;
                        else if (byteRead != FLAG)
                            machineState = START;
                        break;

                    case A:
                        if (byteRead == C_DISC)
                            machineState = C;
                        else if (byteRead == FLAG)
                            machineState = F;
                        break;

                    case C:
                        // também afeta esta
                        if (byteRead == (A_ER ^ C_DISC))
                            machineState = BCC1;
                        else if (byteRead == FLAG)
                            machineState = F;
                        break;

                    case BCC1:
                        if (byteRead == FLAG)
                            machineState = STOP;
                        else
                            machineState = START;
                        break;

                    default:
                        break;
                    }
                }
            }
            retransmissions--;
        }

        if (machineState != STOP)
            return -1;

        // Recetor responde com DISC
        sendSupervisionFrame(fd, A_RE, C_DISC);

        // Recetor aguarda UA do transmissor
        machineState = START;
        while (machineState != STOP && retransmissions > 0)
        {
            if (read(fd, &byteRead, 1) > 0)
            {
                switch (machineState)
                {
                case START:
                    if (byteRead == FLAG)
                        machineState = F;
                    break;

                case F:
                    if (byteRead == A_ER)
                        machineState = A;
                    else if (byteRead != FLAG)
                        machineState = START;
                    break;

                case A:
                    if (byteRead == C_UA)
                        machineState = C;
                    else if (byteRead == FLAG)
                        machineState = F;
                    break;

                case C:
                    // também afeta esta
                    if (byteRead == (A_ER ^ C_UA))
                        machineState = BCC1;
                    else if (byteRead == FLAG)
                        machineState = F;
                    break;

                case BCC1:
                    if (byteRead == FLAG)
                        machineState = STOP;
                    else
                        machineState = START;
                    break;

                default:
                    break;
                }
            }
        }

        if (machineState != STOP)
            return -1;
        break;

    default:
        break;
    }

    // ainda não fiz isto
    if (showStatistics)
        displayStatistics();
    return closeSerialPort();
}

LinkLayerState stateMachine(int frameType)
{
    switch (frameType)
    {
    case 1:
        switch (role)
        {
        case 1:
            /* code */
            break;

        default:
            break;
        }
        break;

    default:
        switch (role)
        {
        case 1:
            break;

        default:
            break;
        }
        break;
    }
}

sendSupervisionFrame(fd, A_byte, C_byte)
{
    unsigned char sFrame[5] = {FLAG, A_byte, C_byte, A_byte ^ C_byte, FLAG};
    return write(fd, sFrame, 5);
}

unsigned char getControlFrame(int fd)
{
    // to do
}

// to do
void displayStatistics()
{
    printf("AQUI TENHO DE PRINTAR AS STATS");
}