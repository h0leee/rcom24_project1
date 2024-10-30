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

    int fd = makeConnection(&connectionParameters);
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        return -1;
    }

    printf("New termio structure set\n");

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
            alarmFired = FALSE;

            while (machineState != STOP && !alarmFired)
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



unsigned char computeBCC2(const unsigned char *buffer, size_t length)
{
    if (buffer == NULL || length == 0)
    {
        printf("Error: Invalid buffer parameters in computeBCC2.\n");
        return 0xFF; // Valor especial para indicar erro
    }

    unsigned char BCC2Value = 0x00;

    // Inicia o cálculo do BCC2 no índice 0
    for (size_t i = 0; i < length; i++)
    {
        BCC2Value ^= buffer[i];
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

    // frame sem qualquer FLAG
    
    frame[0] = A_ER;
    frame[1] = C_INF(txFrame);
    frame[2] = frame[0] ^ frame[1]; // BCC1
    memcpy(&frame[3], buf, bufSize);
    frame[3 + bufSize] = computeBCC2(buf, bufSize); // BCC2

    // Aplicar byte stuffing ao campo de dados e BCC2
    int frameSize = 4 + bufSize; // A_ER + C + BCC1 + Dados + BCC2
    unsigned char stuffedFrame[MAX_PAYLOAD_SIZE];
    int stuffedSize = byteStuffing(frame, frameSize, stuffedFrame);

    // Agora adicionar as FLAGs
    unsigned char finalFrame[MAX_PAYLOAD_SIZE];
    finalFrame[0] = FLAG;
    memcpy(&finalFrame[1], stuffedFrame, stuffedSize);
    finalFrame[stuffedSize + 1] = FLAG;
    int totalSize = stuffedSize + 2; // FLAGS adicionadas


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
    LinkLayerState machineState = START;
    int frameIndex = 0;
    unsigned char bcc2;

    while (1)
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
                    // Se byteRead == FLAG, permanece no estado F
                    break;

                case A:
                    if (byteRead == C_N(0) || byteRead == C_N(1))
                    {
                        machineState = C;
                        cByte = byteRead;
                    }
                    else if (byteRead == FLAG)
                        machineState = F;
                    else if (byteRead == C_DISC)
                    {
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
                    if (byteRead == FLAG)
                    {
                        // Frame inválido, no data
                        machineState = START;
                        frameIndex = 0;
                    }
                    else if (byteRead == ESC)
                    {
                        machineState = ESC_FOUND;
                    }
                    else
                    {
                        // Armazena o byte de dados
                        if (frameIndex < MAX_PAYLOAD_SIZE)
                        {
                            packet[frameIndex++] = byteRead;
                        }
                        else
                        {
                            printf("Erro: buffer de pacote excedido.\n");
                            machineState = START;
                            frameIndex = 0;
                        }
                    }
                    break;

                case ESC_FOUND:
                    machineState = BCC1; // Retorna ao estado BCC1 após tratar o escape

                    if (byteRead == 0x5E) // FLAG escapado
                    {
                        if (frameIndex < MAX_PAYLOAD_SIZE)
                        {
                            packet[frameIndex++] = FLAG;
                        }
                        else
                        {
                            printf("Erro: buffer de pacote excedido.\n");
                            machineState = START;
                            frameIndex = 0;
                        }
                    }
                    else if (byteRead == 0x5D) // ESC escapado
                    {
                        if (frameIndex < MAX_PAYLOAD_SIZE)
                        {
                            packet[frameIndex++] = ESC;
                        }
                        else
                        {
                            printf("Erro: buffer de pacote excedido.\n");
                            machineState = START;
                            frameIndex = 0;
                        }
                    }
                    else
                    {
                        printf("Erro: sequência de escape inválida.\n");
                        machineState = START;
                        frameIndex = 0;
                    }
                    break;

                default:
                    machineState = START;
                    frameIndex = 0;
                    break;
            }

            // Verifica se a FLAG final foi recebida no estado BCC1
            if (machineState == BCC1 && byteRead == FLAG)
            {
                // Recebeu FLAG final
                if (frameIndex < 1)
                {
                    printf("Erro: Nenhum dado recebido antes da FLAG final.\n");
                    machineState = START;
                    frameIndex = 0;
                    continue;
                }

                // O último byte recebido é o BCC2
                bcc2 = packet[frameIndex - 1];
                frameIndex--; // Exclui o BCC2 dos dados

                // Calcula o BCC2 dos dados recebidos
                unsigned char calculatedBCC2 = computeBCC2(packet, frameIndex);

                if (calculatedBCC2 == bcc2)
                {
                    sendSupervisionFrame(fd, A_RE, C_RR(rxFrame));
                    rxFrame = (rxFrame + 1) % 2;
                    return frameIndex; // Retorna o número de bytes de dados recebidos
                }
                else
                {
                    printf("Erro: BCC2 inválido. Necessária retransmissão.\n");
                    sendSupervisionFrame(fd, A_RE, C_REJ(rxFrame));
                    machineState = START;
                    frameIndex = 0;
                    continue;
                }
            }
        }
    }

    return -1; // Erro caso o loop termine sem receber a FLAG final
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

int sendSupervisionFrame(int fd, unsigned char A_byte, unsigned char C_byte)
{
    unsigned char sFrame[5] = {FLAG, A_byte, C_byte, A_byte ^ C_byte, FLAG};
    
    // isto parece estar mesmo mal 
    return write(fd, sFrame, 5);
}



// ela pode estar mal
unsigned char getControlFrame(int fd)
{
    unsigned char byteRead;
    LinkLayerState machineState = START;
    unsigned char controlByte = 0;

    while (machineState != STOP)
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
                else if (byteRead != FLAG)
                    machineState = START;
                break;

            case A:
                if (byteRead == C_RR(0) || byteRead == C_RR(1) || byteRead == C_REJ(0) || byteRead == C_REJ(1))
                {
                    controlByte = byteRead;
                    machineState = C;
                }
                else if (byteRead == FLAG)
                    machineState = F;
                else
                    machineState = START;
                break;

            case C:
                if (byteRead == (A_RE ^ controlByte))
                    machineState = BCC1;
                else if (byteRead == FLAG)
                    machineState = F;
                else
                    machineState = START;
                break;

            case BCC1:
                if (byteRead == FLAG)
                    machineState = STOP;
                else
                    machineState = START;
                break;

            default:
                machineState = START;
                break;
            }
        }
    }
    return controlByte;
}


// to do
void displayStatistics()
{
    printf("AQUI TENHO DE PRINTAR AS STATS");
}