
#include "link_layer.h"
#include "serial_port.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>

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

int alarmFired = FALSE;
int alarmCount = 0;
int numberRetransmissions = 0;
int timeOut = 0;
LinkLayerState machineState;
LinkLayerRole role;
int fd;
int txFrame = 0, rxFrame = 1;

void cleanBuffer(unsigned char* buffer, int bufferSize, int* dataSize) {
    if (buffer == NULL || dataSize == NULL || bufferSize <= 0) {
        return;
    }

    memset(buffer, 0, bufferSize);
    *dataSize = 0;
}


int sendRejectionFrame(int fd, unsigned char address, unsigned char control)
{
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;             // C_REJ(0) ou C_REJ(1)
    frame[3] = frame[1] ^ frame[2]; // BCC
    frame[4] = FLAG;

    if (writeBytesSerialPort(frame, 5) < 0)
        return -1;
    return 0;
}

int sendSupervisionFrame(int fd, unsigned char address, unsigned char control)
{
    printf("[sendSupervisionFrame] Iniciando envio de frame de supervisão.\n");
    printf("[sendSupervisionFrame] Address: 0x%02X, Control: 0x%02X\n", address, control);

    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;             // C_REJ(0) ou C_REJ(1)
    frame[3] = frame[1] ^ frame[2]; // BCC
    frame[4] = FLAG;

    printf("[sendSupervisionFrame] Frame a ser enviado: ");
    for (int i = 0; i < 5; i++)
    {
        printf("0x%02X ", frame[i]);
    }
    printf("\n");

    int writeResult = writeBytesSerialPort(frame, 5);
    if (writeResult < 0)
    {
        printf("[sendSupervisionFrame] Erro ao escrever bytes na porta serial.\n");
        return -1;
    }
    printf("[sendSupervisionFrame] Frame enviado com sucesso.\n");
    return 0;
}

// Alarm function handler
// Alarm function handler
void alarmHandler(int signal)
{
    printf("[alarmHandler] Sinal recebido: %d\n", signal);
    alarmFired = TRUE;  // Deve ser TRUE para sinalizar que o alarme foi disparado
    alarmCount++;
    printf("[alarmHandler] Alarme disparado número: %d\n", alarmCount);
    printf("[alarmHandler] Alarm #%d executado.\n", alarmCount);
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    printf("[llopen] Iniciando llopen com os seguintes parâmetros:\n");
    printf("  Serial Port: %s\n", connectionParameters.serialPort);
    printf("  Baud Rate: %d\n", connectionParameters.baudRate);
    printf("  Role: %s\n", connectionParameters.role == LlTx ? "LlTx (Transmissor)" : "LlRx (Receptor)");
    printf("  Retransmissões: %d\n", connectionParameters.nRetransmissions);
    printf("  Timeout: %d\n", connectionParameters.timeout);

    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0)
    {
        perror("[llopen] Erro ao abrir a porta serial");
        return -1;
    }

    printf("[llopen] Configuração da porta serial concluída e buffers limpos\n");

    machineState = START;
    unsigned char receivedByte;
    timeOut = connectionParameters.timeout;
    numberRetransmissions = connectionParameters.nRetransmissions;
    role = connectionParameters.role;

    if (role == LlTx) {
        printf("[llopen] Modo Transmissor\n");
        signal(SIGALRM, alarmHandler);

        int attempts = 0;
        while (attempts < numberRetransmissions) {
            printf("[llopen] Tentativa %d de %d\n", attempts + 1, numberRetransmissions);
            machineState = START;
            sendSupervisionFrame(fd, A_ER, C_SET);
            printf("[llopen] Transmissor enviou SET\n");

            alarmFired = FALSE;  // Garante que o alarme não esteja "ativo" no início de cada tentativa
            alarm(timeOut);
            printf("[llopen] Alarme configurado para %d segundos\n", timeOut);

            while (machineState != STOP && !alarmFired) {
                receivedByte = 0; // Limpar byte recebido

                int bytesRead = readByteSerialPort(&receivedByte);
                if (bytesRead > 0) {
                    printf("[llopen][Transmissor] Byte recebido: 0x%02X | Estado atual: %d\n", receivedByte, machineState);

                    switch (machineState) {
                    case START:
                        if (receivedByte == FLAG) machineState = F;
                        break;
                    case F:
                        if (receivedByte == A_RE) machineState = A;
                        else if (receivedByte != FLAG) machineState = START;
                        break;
                    case A:
                        if (receivedByte == C_UA) machineState = C;
                        else machineState = START;
                        break;
                    case C:
                        if (receivedByte == (A_RE ^ C_UA)) machineState = BCC1;
                        else machineState = START;
                        break;
                    case BCC1:
                        if (receivedByte == FLAG) machineState = STOP;
                        else machineState = START;
                        break;
                    default:
                        machineState = START;
                        break;
                    }
                } else if (bytesRead == 0) {
                    // printf("[llopen][Transmissor] Nenhum byte lido\n");
                } else {
                    perror("[llopen][Transmissor] Erro ao ler byte");
                    break;
                }
            }

            alarm(0);
            printf("[llopen][Transmissor] Alarme cancelado\n");

            if (machineState == STOP) {
                printf("[llopen][Transmissor] Quadro UA recebido com sucesso\n");
                return fd; // Conexão estabelecida
            } else if (alarmFired) {
                printf("[llopen][Transmissor] Timeout ocorreu após %d segundos\n", timeOut);
            }

            attempts++;
        }

        printf("[llopen][Transmissor] Número máximo de retransmissões atingido. Abortando conexão.\n");
        return -1;
    }

    else if (role == LlRx)
    {
        printf("[llopen] Modo Receptor\n");
        printf("[llopen][Receptor] Aguardando quadro SET...\n");

        machineState = START;

        while (machineState != STOP)
        {
            receivedByte = 0;

            int bytesRead = readByteSerialPort(&receivedByte);

            if (bytesRead > 0)
            {
                switch (machineState)
                {
                case START:
                    if (receivedByte == FLAG)
                        machineState = F;
                    else
                        machineState = START;
                    break;
                case F:
                    if (receivedByte == A_ER)
                        machineState = A;
                    else if (receivedByte == FLAG)
                        machineState = F;
                    else
                        machineState = START;
                    break;
                case A:
                    if (receivedByte == C_SET)
                        machineState = C;
                    else
                        machineState = START;
                    break;
                case C:
                    if (receivedByte == (A_ER ^ C_SET))
                        machineState = BCC1;
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
            else if (bytesRead < 0)
            {
                perror("[llopen][Receptor] Erro ao ler byte");
                return -1;
            }
        }

        printf("[llopen][Receptor] Quadro SET recebido com sucesso\n");
        sendSupervisionFrame(fd, A_RE, C_UA);
        printf("[llopen][Receptor] Quadro UA enviado\n");
        return fd;
    }
    printf("[llopen] Erro: Role desconhecido na conexão.\n");
    return -1;
}


int byteStuffing(const unsigned char *inputMsg, int inputSize, unsigned char *outputMsg, unsigned char bcc2)
{
    int stuffedSize = 0;

    printf("MAX_PAYLOAD_SIZE: %d\n", MAX_PAYLOAD_SIZE);
    printf("input message size: %d\n", inputSize);

    for (int i = 0; i < inputSize; i++)
    {
        printf("[STUFFING NO INDEX %d]\n", i);
        if (inputMsg[i] == FLAG || inputMsg[i] == ESC)
        {
            if (stuffedSize + 2 > MAX_PAYLOAD_SIZE)
            {
                fprintf(stderr, "Output buffer overflow in byteStuffing\n");
                return -1;
            }
            outputMsg[stuffedSize++] = ESC;
            outputMsg[stuffedSize++] = inputMsg[i] ^ 0x02;
        }
        else
        {
            if (stuffedSize + 1 > MAX_PAYLOAD_SIZE)
            {
                fprintf(stderr, "Output buffer overflow in byteStuffing\n");
                return -1;
            }
            outputMsg[stuffedSize++] = inputMsg[i];
        }
    }

    if (bcc2 == FLAG || bcc2 == ESC)
    {
        if (stuffedSize + 2 > MAX_PAYLOAD_SIZE)
        {
            fprintf(stderr, "Output buffer overflow in byteStuffing for BCC2\n");
            return -1;
        }
        outputMsg[stuffedSize++] = ESC;
        outputMsg[stuffedSize++] = bcc2 ^ 0x02;
    }
    else
    {
        if (stuffedSize + 1 > MAX_PAYLOAD_SIZE)
        {
            fprintf(stderr, "Output buffer overflow in byteStuffing for BCC2\n");
            return -1;
        }
        outputMsg[stuffedSize++] = bcc2;
    }

    return stuffedSize;
}

// função usada para reverter o byte stuffing aplicado na mensagem
int byteDestuffing(const unsigned char *inputMsg, int inputSize, unsigned char *outputMsg, int maxOutputSize)
{
    int destuffedSize = 0;
    int i = 0;

    while (i < inputSize)
    {
        if (destuffedSize >= maxOutputSize)
        {
            fprintf(stderr, "Output buffer overflow in byteDestuffing\n");
            return -1;
        }
        if (inputMsg[i] == ESC)
        {
            if (i + 1 >= inputSize)
            {
                fprintf(stderr, "Incomplete escape sequence\n");
                return -1;
            }
            outputMsg[destuffedSize++] = inputMsg[i + 1] ^ 0x20;
            i += 2;
        }
        else
        {
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
    if (bufSize * 2 > MAX_PAYLOAD_SIZE) {
        fprintf(stderr, "Erro: Payload excede o tamanho máximo permitido após byte stuffing\n");
        return -1;
    }

    unsigned char bcc2 = computeBCC2(buf, bufSize); // Calcula BCC2 sem stuffing

    unsigned char stuffedPayload[MAX_PAYLOAD_SIZE];
    int stuffedPayloadSize = byteStuffing(buf, bufSize, stuffedPayload, bcc2);

    if (stuffedPayloadSize == -1) {
        fprintf(stderr, "Erro no byte stuffing\n");
        return -1;
    }

    int totalSize = stuffedPayloadSize + 5; // Recalcula totalSize baseado no stuffedPayloadSize
    printf("Stuffed Payload Size: %d\n", stuffedPayloadSize);
    printf("Total Size (with headers): %d\n", totalSize);

    if (totalSize > MAX_PAYLOAD_SIZE) {
        fprintf(stderr, "Erro: Total size após stuffing excede o limite\n");
        return -1;
    }

    // Aloca memória dinamicamente para fullFrame
    unsigned char *fullFrame = malloc(totalSize);
    if (fullFrame == NULL) {
        fprintf(stderr, "Erro de alocação de memória para fullFrame\n");
        return -1;
    }

    // Prepara o cabeçalho da trama
    fullFrame[0] = FLAG;
    fullFrame[1] = A_ER;
    fullFrame[2] = C_N(txFrame);
    fullFrame[3] = fullFrame[1] ^ fullFrame[2];

    // Copia o payload "stuffed" para a frame completa
    for (int i = 0; i < stuffedPayloadSize; i++) {
        fullFrame[i + 4] = stuffedPayload[i];
    }

    // Define o FLAG final da frame
    fullFrame[4 + stuffedPayloadSize] = FLAG;

    int attemptCounter = 0;
    int successfulTransmission = 0;
    int failedTransmission = 0;

    while (attemptCounter < numberRetransmissions) {
        cleanBuffer(fullFrame, totalSize, NULL);

        alarmFired = FALSE;
        alarm(timeOut);

        printf("Tentativa de transmissão #%d\n", attemptCounter + 1);

        int bytesWritten = writeBytesSerialPort(fullFrame, totalSize);
        if (bytesWritten < 0) {
            perror("Erro ao escrever o frame");
            attemptCounter++;
            continue;
        }

        while (!successfulTransmission && !alarmFired && !failedTransmission) {
            int result = getControlFrame(fd);
            if (result == C_REJ(0) || result == C_REJ(1)) {
                failedTransmission = 1;
            } else if (result == C_RR(0) || result == C_RR(1)) {
                successfulTransmission = 1;
                txFrame = (txFrame + 1) % 2;
            }
        }

        if (successfulTransmission) break;

        attemptCounter++;
    }

    // Limpa a memória alocada para evitar vazamento
    free(fullFrame);

    if (!successfulTransmission) {
        printf("Número máximo de tentativas atingido\n");
        llclose(fd);
        return -1;
    }

    printf("Dados aceites com sucesso!\n");
    return 0;
}
////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////

// START, A, C,  se aparecer uma merda random tenho de mandar o grande rej
int llread(unsigned char *packet)
{
    unsigned char receivedByte, cByte;
    LinkLayerState machineState = START;
    int frameIndex = 0;
    unsigned char bcc2;

    while (1)
    {
        // vou ter de alterar aqui
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
                if (receivedByte == C_N(0) || receivedByte == C_N(1))
                {
                    machineState = C;
                    cByte = receivedByte;
                }
                else if (receivedByte == FLAG)
                    machineState = F;
                else if (receivedByte == C_DISC)
                {
                    sendSupervisionFrame(fd, A_RE, C_DISC);
                    return 0;
                }
                else
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
                if (receivedByte == FLAG)
                {
                    if (frameIndex < 1)
                    {
                        printf("Erro: Nenhum dado recebido antes da FLAG final.\n");
                        machineState = START;
                        frameIndex = 0;
                        continue;
                    }

                    bcc2 = packet[frameIndex - 1];
                    frameIndex--;
                    packet[frameIndex] = '\0'; // para anular o bcc2 no packet

                    unsigned char calculatedBCC2 = computeBCC2(packet, frameIndex);

                    if (calculatedBCC2 == bcc2)
                    {
                        sendSupervisionFrame(fd, A_RE, C_RR(rxFrame));
                        rxFrame = (rxFrame + 1) % 2;
                        return frameIndex;
                    }
                    else
                    {
                        cleanBuffer(packet, MAX_PAYLOAD_SIZE, &frameIndex);

                        if (tcflush(fd, TCIFLUSH) == -1)
                        {
                            perror("falha ao limpar buffer de serial port");
                        }

                        printf("Erro: BCC2 inválido. Necessária retransmissão.\n");
                        sendSupervisionFrame(fd, A_RE, C_REJ(rxFrame));
                        machineState = START;
                        frameIndex = 0;
                        continue;
                    }
                }
                else if (frameIndex >= MAX_PAYLOAD_SIZE)
                {
                    perror("[LLREAD] Overflow no buffer de dados");
                    sendSupervisionFrame(fd, A_RE, C_REJ(rxFrame));
                    cleanBuffer(packet, MAX_PAYLOAD_SIZE, &frameIndex);
                    machineState = START;
                    continue;
                }
                else if (receivedByte == ESC)
                {
                    machineState = ESC_FOUND;
                }
                else
                {
                    packet[frameIndex++] = receivedByte;
                }
                break;

            case ESC_FOUND:
                machineState = BCC1;
                if (receivedByte == 0x5E)
                {
                    packet[frameIndex++] = FLAG;
                }
                else if (receivedByte == 0x5D)
                {
                    packet[frameIndex++] = ESC;
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
        }
    }

    return -1;
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

    if (tcflush(fd, TCIOFLUSH) == -1)
    {
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