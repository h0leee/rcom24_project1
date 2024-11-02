#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_PAYLOAD_SIZE 1000
#define PACKET_SIZE 256

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
        if (llopen(connectionParameters) >= 0) {
            printf("Iniciando transmissão\n");

            FILE *file = fopen(filename, "rb");
            if (!file) {
                perror("Erro ao abrir o arquivo");
                llclose(1);
                return;
            }

            fseek(file, 0, SEEK_END);
            long fileSize = ftell(file);
            fseek(file, 0, SEEK_SET);

            unsigned char *buffer = malloc(fileSize);
            if (!buffer) {
                perror("Erro ao alocar memória");
                fclose(file);
                llclose(1);
                return;
            }

            size_t bytesRead = fread(buffer, 1, fileSize, file);
            if (bytesRead != fileSize) {
                perror("Erro ao ler o arquivo");
                free(buffer);
                fclose(file);
                llclose(1);
                return;
            }
            fclose(file);

            for (int i = 0; i < fileSize; i += PACKET_SIZE) {
                int currentPacketSize = (i + PACKET_SIZE > fileSize) ? fileSize - i : PACKET_SIZE;
                printf("Enviando pacote de tamanho %d\n", currentPacketSize);
                
                if (llwrite(&buffer[i], currentPacketSize) < 0) {
                    printf("Erro ao enviar pacote\n");
                    free(buffer);
                    llclose(1);
                    return;
                }
            }

            free(buffer);
            llclose(1);
            printf("Transmissão concluída\n");
        }
    } else if (strcmp(role, "rx") == 0) {
        connectionParameters.role = LlRx;
        if (llopen(connectionParameters) >= 0) {
            printf("Recebendo dados...\n");

            unsigned char buffer[MAX_PAYLOAD_SIZE];
            int totalBytesRead = 0;
            int bytesRead;

            FILE *outputFile = fopen(filename, "wb");
            if (!outputFile) {
                perror("Erro ao abrir arquivo de saída");
                llclose(1);
                return;
            }

            while ((bytesRead = llread(buffer)) > 0) {
                printf("Pacote recebido de %d bytes\n", bytesRead);
                fwrite(buffer, 1, bytesRead, outputFile);
                totalBytesRead += bytesRead;
            }

            fclose(outputFile);

            if (bytesRead < 0) {
                printf("Erro ao ler os dados\n");
            } else {
                printf("Total de bytes recebidos: %d\n", totalBytesRead);
                printf("Dados salvos no arquivo '%s'\n", filename);
            }

            llclose(1);
        }
    } else {
        printf("Função de transmissão inválida: %s\n", role);
    }
}
