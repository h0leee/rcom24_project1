// Link layer protocol implementation

// depois poderei tentar modular o código da stateMachine
// stateMachine, getControlPacket, displayStatistics
// bcc2 sem stuffing e depois colocar stuffing nele !!!
// limpar sempre o buffer, SEMPRE MESMO
    // memset(0, buffer, bufferSize)
    // bufferSize = 0
// no read, o receiver tem de perceber que pode receber disc, se ultrapassar o limite de transmissões

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

    if(tcflush(fd, TCIOFLUSH ) == -1) {
        perror("Falgha ao limpar buffer da serial port");
    }

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

    if (tcflush(fd, TCIOFLUSH) == -1) { // limpar tudo antes de iniciar
        perror("Falha ao limpar os buffers da porta serial");
    }

    printf("New termio structure set\n");

    machineState = START;
    unsigned char receivedByte;
    timeOut = connectionParameters.timeout;
    numberRetransmissions = connectionParameters.nRetransmissions; // Usar uma cópia local para preservar o valor original

    switch (connectionParameters.role)
    {
    case LlTx:

        printf("transmissor a iniciar conexão");

        (void)signal(SIGALRM, alarmHandler);

        int retransmissions = numberRetransmissions;

        while (retransmissions > 0)
        {

            machineState = START;
            sendSupervisionFrame(fd, A_ER, C_SET);
            printf("transmissor envio SET");

            alarmFired = FALSE;
            alarm(timeOut);
            

            while (machineState != STOP && !alarmFired)
            {
                if (readByteSerialPort(&receivedByte) > 0)
                {
                    printf("Transmissor: byte recebido: 0x%02X\n", receivedByte);

                    switch (machineState)
                    {
                    case START:
                        if (receivedByte == FLAG)
                            machineState = F;
                        break;
                    case F:
                        if (receivedByte == A_RE)
                            machineState = A;
                        else if(receivedByte == FLAG) machineState = F;
                        else machineState = START;
                        break;
                    case A:
                        if (receivedByte == C_UA)
                            machineState = C;
                        else if(receivedByte == FLAG) machineState = F;
                        else machineState = START;
                        break;
                    case C:
                        if (receivedByte == (A_RE ^ C_UA))
                            machineState = BCC1;
                        else {
                            printf("Transmissor: BCC1 incorreto. Descartando quadro.\n");
                            machineState = START;
                        }
                        break;
                    case BCC1:
                        if (receivedByte == FLAG)
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

            alarm(0);

            if(machineState == STOP) {
                printf("Transmissor: Quadro UA recebido com sucesso\n");
                break;
            }

            else if(alarmFired) {
                printf("Transmissor: timeout. Vamos tentar de novo\n");
                retransmissions--;
            }

            else{
                printf("Transmissor: erro a receber UA\n");
                retransmissions--;
            }

        }

        if (retransmissions == 0) {
            printf("Transmissor: número máximo de retransmissões atingido. Abortando conexão.\n");
            return -1;
        }
        break;

    case LlRx:
    {
        printf("Receptor: aguardando quadro SET...\n");

        // Configura o manipulador de sinal para o alarme
        (void)signal(SIGALRM, alarmHandler);

        // Define o timeout para o receptor (por exemplo, 3 segundos)
        int receiverTimeout = 3;
        alarmFired = FALSE;
        alarm(receiverTimeout);

        int errorCount = 0;
        const int maxErrors = 5;

        while (machineState != STOP && !alarmFired && errorCount < maxErrors)
        {
            if (readByteSerialPort(&receivedByte) > 0)
            {
                printf("Receptor: byte recebido: 0x%02X\n", receivedByte);

                switch (machineState)
                {
                case START:
                    if (receivedByte == FLAG)
                        machineState = F;
                    break;
                case F:
                    if (receivedByte == A_ER)
                        machineState = A;
                    else if (receivedByte != FLAG)
                        machineState = START;
                    break;
                case A:
                    if (receivedByte == C_SET)
                        machineState = C;
                    else if (receivedByte == FLAG)
                        machineState = F;
                    else
                        machineState = START;
                    break;
                case C:
                    if (receivedByte == (A_ER ^ C_SET))
                        machineState = BCC1;
                    else
                    {
                        printf("Receptor: BCC1 incorreto. Descartando quadro.\n");
                        machineState = START;
                        errorCount++;
                    }
                    break;
                case BCC1:
                    if (receivedByte == FLAG)
                        machineState = STOP;
                    else
                        machineState = START;
                    break;
                default:
                    printf("Receptor: estado desconhecido.\n");
                    machineState = START;
                    errorCount++;
                    break;
                }
            }
            else
            {
                printf("Receptor: falha na leitura do byte.\n");
                errorCount++;
            }
        }

        alarm(0); // Cancela o alarme

        if (alarmFired)
        {
            printf("Receptor: timeout ao esperar pelo quadro SET.\n");
            return -1;
        }

        if (errorCount >= maxErrors)
        {
            printf("Receptor: número máximo de erros atingido. Abortando conexão.\n");
            return -1;
        }

        printf("Receptor: quadro SET recebido com sucesso.\n");

        // Envia o quadro UA em resposta
        sendSupervisionFrame(fd, A_RE, C_UA);
        printf("Receptor: quadro UA enviado.\n");

        break;
    }
    default:
        printf("Erro: papel desconhecido na conexão.\n");
        return -1;
    }

    return fd;
}

/////////////////////////////////////////////////////
// STUFFING
/////////////////////////////////////////////////////

// Stuffing e o processo inverso vao ser usados no LLWRITE


int byteStuffing(const unsigned char *inputMsg, int inputSize, unsigned char* outputMsg, unsigned char bcc2) {
    int stuffedSize = 0;

    printf("\nSTUFFING STARTED\n"); // Debug

    for (int i = 0; i < inputSize; i++) {
        if (inputMsg[i] == FLAG || inputMsg[i] == ESC) {
            // Precisa de 2 bytes para stuffing
            if (stuffedSize + 2 > MAX_PAYLOAD_SIZE) {
                fprintf(stderr, "Output buffer overflow in byteStuffing\n");
                return -1;
            }
            outputMsg[stuffedSize++] = ESC;
            outputMsg[stuffedSize++] = inputMsg[i] ^ 0x02;
        } else {
            // Precisa de 1 byte
            if (stuffedSize + 1 > MAX_PAYLOAD_SIZE) {
                fprintf(stderr, "Output buffer overflow in byteStuffing\n");
                return -1;
            }
            outputMsg[stuffedSize++] = inputMsg[i];
        }
    }

    if(bcc2 == FLAG || bcc2 == ESC) {
        if (stuffedSize + 2 > MAX_PAYLOAD_SIZE) {
                fprintf(stderr, "Output buffer overflow in byteStuffing\n");
                return -1;
            }
            outputMsg[stuffedSize++] = ESC;
            outputMsg[stuffedSize++] = bcc2 ^ 0x02;
    }

    return stuffedSize;
}





// função usada para reverter o byte stuffing aplicado na mensagem
int byteDestuffing(const unsigned char *inputMsg, int inputSize, unsigned char *outputMsg, int maxOutputSize) {
    int destuffedSize = 0;
    int i = 0;

    while (i < inputSize) {
        if (destuffedSize >= maxOutputSize) {
            fprintf(stderr, "Output buffer overflow in byteDestuffing\n");
            return -1;
        }
        if (inputMsg[i] == ESC) {
            if (i + 1 >= inputSize) {
                fprintf(stderr, "Incomplete escape sequence\n");
                return -1;
            }
            outputMsg[destuffedSize++] = inputMsg[i + 1] ^ 0x20;
            i += 2;
        } else {
            outputMsg[destuffedSize++] = inputMsg[i++];
        }
    }
    return destuffedSize;
}



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

int llwrite(const unsigned char *buf, int bufSize) {
    // no llwrite recebo um packet ou a parte da payload de uma frame apenas? 

    // antes de chamar construção, eu deveria saber qual o tamanho 
    // o bcc2 tmb pode ser necessário dar stuff
    // mas o payload é de certeza 

    
    unsigned char bcc2 =  computeBCC2(buf, bufSize); // BCC2, sem byteStuffing 


    unsigned char stuffedPayload[bufSize*2 + 2]; // i know, exagerado, mas assim não há erros, este +2 é para bcc2
    int stuffedPayloadSize = byteStuffing(buf, bufSize, stuffedPayload, bcc2); // aqui terei a length do bytestuffing da payload com bcc2

    unsigned char fullFrame[stuffedPayloadSize + 5]; // stuffed e 5 bytes adicionais (f, a, c, bcc1, f)
    int totalSize = stuffedPayloadSize + 4;
    unsigned char fullFrame[totalSize];

    int attemptCounter = 0;
    int successfulTransmission = 0;
    int failedTransmission = 0;

    while (attemptCounter < numberRetransmissions) {
        // Limpa o stuffedFrame antes de cada tentativa de stuffing
        // está exagerado este size, i know 
        cleanBuffer(fullFrame, totalSize, NULL);
        cleanBuffer(stuffedPayload, stuffedPayloadSize, NULL);

        fullFrame[0] = F;
        fullFrame[1] = A_ER;
        fullFrame[2] = C_N(txFrame);
        fullFrame[3] = fullFrame[1] ^ fullFrame[2];


        // Executa o byte stuffing no payload e no bcc2
        stuffedPayloadSize = byteStuffing(buf, bufSize, stuffedPayload, bcc2);
        if (stuffedPayloadSize == -1) {
            fprintf(stderr, "Erro no byte stuffing\n");
            return -1;
        }


        for (size_t i = 4; i < stuffedPayloadSize + 4; i++)
        {
            fullFrame[i] = stuffedPayload[i-4];
        }

        fullFrame[4 + stuffedPayloadSize] = FLAG;

        STOP = FALSE;
        alarmFired = FALSE;
        alarm(timeOut);               // Define o temporizador

        int bytesWritten = writeBytesSerialPort(fullFrame, totalSize); // isto pode estar mal 
        if (bytesWritten < 0) {
            perror("Erro ao escrever o frame");
            continue; // Tenta novamente
        }

        // Verifica se a transmissão foi bem-sucedida
        while (!successfulTransmission && !alarmFired && !failedTransmission) {
            int result = getControlFrame(fd);
            if (result == C_REJ(0) || result == C_REJ(1)) {
                failedTransmission = 1;
            }
            else if (result == C_RR(0) || result == C_RR(1)) {
                successfulTransmission = 1;
                txFrame = (txFrame + 1) % 2; // Alterna o número de sequência
            }
        }

        if (successfulTransmission) break;
        attemptCounter++;
    }

    cleanBuffer(fullFrame, totalSize, NULL);
    cleanBuffer(stuffedPayload, stuffedPayloadSize, NULL);

    if (!successfulTransmission) {
        printf("Número máximo de tentativas atingido\n");
        llclose(fd);
        return -1;
    }

    printf("Dados aceites com sucesso!\n"); // Debug
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    unsigned char receivedByte, cByte; // cByte corresponde ao C
    LinkLayerState machineState = START;
    int frameIndex = 0; // index do packet que estou a ler
    unsigned char bcc2; // para depois ter método de comparação

    while (1) {
        if (readByteSerialPort(&receivedByte) > 0) {

            switch (machineState) {
                case START:
                    if (receivedByte == FLAG)
                        machineState = F;
                    break;

                case F:
                    if (receivedByte == A_ER)
                        machineState = A;
                    else if (receivedByte != FLAG)
                        machineState = START;
                    break;

                case A:
                    if (receivedByte == C_N(0) || receivedByte == C_N(1)) {
                        machineState = C;
                        cByte = receivedByte;
                    } else if (receivedByte == FLAG)
                        machineState = F;
                    else if (receivedByte == C_DISC) {
                        sendSupervisionFrame(fd, A_RE, C_DISC);
                        return 0; // Indica desconexão
                    } else
                        machineState = START;
                    break;

                case C:
                    if (receivedByte == (A_ER ^ cByte))
                        machineState = BCC1;
                    else if (receivedByte == FLAG)
                        machineState = F;
                    else
                        machineState = START;
                    break;

                case BCC1:
                    if (receivedByte == FLAG) {
                        // Frame inválido, no data
                        machineState = F;
                        frameIndex = 0;
                    } else if (frameIndex >= MAX_PAYLOAD_SIZE) {
                        perror("[LLREAD] OVERFLOW NO BUFFER DE DADOS");
                        sendSupervisionFrame(fd, A_RE, C_REJ(rxFrame));
                        clean_buffer(packet, MAX_PAYLOAD_SIZE, &frameIndex);
                        machineState = START;
                        continue;
                    } else if (receivedByte == ESC) {
                        machineState = ESC_FOUND;
                    } else {
                        packet[frameIndex++] = receivedByte;
                    }
                    break;

                case ESC_FOUND:
                    machineState = BCC1;
                    if (receivedByte == 0x5E) { // FLAG escapado
                        packet[frameIndex++] = FLAG;
                    } else if (receivedByte == 0x5D) { // ESC escapado
                        packet[frameIndex++] = ESC;
                    } else {
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

            if (machineState == BCC1 && receivedByte == FLAG) {
                if (frameIndex < 1) {
                    printf("Erro: Nenhum dado recebido antes da FLAG final.\n");
                    machineState = START;
                    frameIndex = 0;
                    continue;
                }

                bcc2 = packet[frameIndex - 1];
                frameIndex--;

                unsigned char calculatedBCC2 = computeBCC2(packet, frameIndex);

                if (calculatedBCC2 == bcc2) {
                    sendSupervisionFrame(fd, A_RE, C_RR(rxFrame));
                    rxFrame = (rxFrame + 1) % 2;
                    return frameIndex;
                } else {
                    clean_buffer(packet, MAX_PAYLOAD_SIZE, &frameIndex);

                    if (tcflush(fd, TCIFLUSH) == -1) {
                        perror("falha ao limpar buffer de serial port");
                    }

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
    unsigned char receivedByte;
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
                if (readByteSerialPort(&receivedByte) > 0)
                {
                    switch (machineState)
                    {
                    case START:
                        if (receivedByte == FLAG)
                            machineState = F;
                        break;

                    case F:
                        if (receivedByte == A_ER)
                            machineState = A;
                        else if (receivedByte != FLAG)
                            machineState = START;
                        break;

                    case A:
                        if (receivedByte == C_DISC)
                            machineState = C;
                        else if (receivedByte == FLAG)
                            machineState = F;
                        break;

                    case C:
                        if (receivedByte == (A_ER ^ C_DISC))
                            machineState = BCC1;
                        else if (receivedByte == FLAG)
                            machineState = F;
                        break;

                    case BCC1:
                        if (receivedByte == FLAG)
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
                if (readByteSerialPort(&receivedByte) > 0)
                {
                    switch (machineState)
                    {
                    case START:
                        if (receivedByte == FLAG)
                            machineState = F;
                        break;

                    case F:
                        // esta cena não sei
                        if (receivedByte == A_ER)
                            machineState = A;
                        else if (receivedByte != FLAG)
                            machineState = START;
                        break;

                    case A:
                        if (receivedByte == C_DISC)
                            machineState = C;
                        else if (receivedByte == FLAG)
                            machineState = F;
                        break;

                    case C:
                        // também afeta esta
                        if (receivedByte == (A_ER ^ C_DISC))
                            machineState = BCC1;
                        else if (receivedByte == FLAG)
                            machineState = F;
                        break;

                    case BCC1:
                        if (receivedByte == FLAG)
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
            if (readByteSerialPort(&receivedByte) > 0)
            {
                switch (machineState)
                {
                case START:
                    if (receivedByte == FLAG)
                        machineState = F;
                    break;

                case F:
                    if (receivedByte == A_ER)
                        machineState = A;
                    else if (receivedByte != FLAG)
                        machineState = START;
                    break;

                case A:
                    if (receivedByte == C_UA)
                        machineState = C;
                    else if (receivedByte == FLAG)
                        machineState = F;
                    break;

                case C:
                    // também afeta esta
                    if (receivedByte == (A_ER ^ C_UA))
                        machineState = BCC1;
                    else if (receivedByte == FLAG)
                        machineState = F;
                    break;

                case BCC1:
                    if (receivedByte == FLAG)
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

    if(tcflush(fd, TCIOFLUSH) == -1) {
        perror("falha ao limpar os buffers do serial port");
        return -1; // falhou
    }

    // ainda não fiz isto
    if (showStatistics)
        displayStatistics();
    return closeSerialPort();
}



// ela pode estar mal
unsigned char getControlFrame(int fd)
{
    unsigned char receivedByte;
    LinkLayerState machineState = START;
    unsigned char controlByte = 0;

    while (machineState != STOP)
    {
        // será que tenho de colcoar uma condição para quando retorna 0 bytes escritos?
        if (readByteSerialPort(&receivedByte) > 0)
        {
            switch (machineState)
            {
            case START:
                if (receivedByte == FLAG)
                    machineState = F;
                break;

            case F:
                if (receivedByte == A_RE)
                    machineState = A;
                else if (receivedByte != FLAG)
                    machineState = START;
                break;

            case A:
                if (receivedByte == C_RR(0) || receivedByte == C_RR(1) || receivedByte == C_REJ(0) || receivedByte == C_REJ(1))
                {
                    controlByte = receivedByte;
                    machineState = C;
                }
                else if (receivedByte == FLAG)
                    machineState = F;
                else
                    machineState = START;
                break;

            case C:
                if (receivedByte == (A_RE ^ controlByte))
                    machineState = BCC1;
                else if (receivedByte == FLAG)
                    machineState = F;
                else
                    machineState = START;
                break;

            case BCC1:
                if (receivedByte == FLAG)
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