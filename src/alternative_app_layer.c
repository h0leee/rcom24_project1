#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_PAYLOAD_SIZE 1000
#define DATA_PACKET_CONTROL 0x02

const char *filename2 = "penguin_received.gif";
void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    filename = filename2;
    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
        if (llopen(connectionParameters) > 0) {
            printf("Iniciando transmissão\n");

            // Abrir o arquivo de imagem
            FILE *file = fopen(filename, "rb");
            printf("Tentando abrir o arquivo: %s\n", filename);
            if (!file) {
                perror("Erro ao abrir o arquivo");
                return;
            }

            fseek(file, 0, SEEK_END);
            long fileSize = ftell(file);
            fseek(file, 0, SEEK_SET);

            unsigned char *buffer = malloc(fileSize); // Alocar buffer para o tamanho completo do arquivo
            if (!buffer) {
                perror("Erro ao alocar memória");
                fclose(file);
                return;
            }

            // Ler a parte superior da imagem no buffer
            size_t bytesRead = fread(buffer, 1, fileSize, file);
            if (bytesRead != fileSize) {
                perror("Erro ao ler o arquivo");
                free(buffer);
                fclose(file);
                return;
            }
            fclose(file);

            // Enviar os pacotes de dados pequenos
            int packetSize = 256;
            for (int i = 0; i < fileSize; i += packetSize) {
                int currentPacketSize = (i + packetSize > fileSize) ? fileSize - i : packetSize;
                printf("Enviando pacote de tamanho %d\n", currentPacketSize);
                // Chamar llwrite para enviar o pacote atual
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
            return;
        }
    } else if (strcmp(role, "rx") == 0) {
        connectionParameters.role = LlRx;
        if (llopen(connectionParameters) > 0) {
            printf("Recebendo dados...\n");

            unsigned char buffer[MAX_PAYLOAD_SIZE];
            int totalBytesRead = 0;
            int bytesRead;

            // Abrir arquivo de saída para salvar os dados recebidos
            //FILE *outputFile = fopen("penguin-received.gif", "wb");
            FILE *outputFile = fopen("teste-received.txt", "wb");
            if (!outputFile) {
                perror("Erro ao abrir arquivo de saída");
                return;
            }

            while ((bytesRead = llread(buffer)) > 0) {
                printf("Pacote recebido de %d bytes\n", bytesRead);
                fwrite(buffer, 1, bytesRead, outputFile);  // Escreve no arquivo
                totalBytesRead += bytesRead;
            }

            fclose(outputFile);

            if (bytesRead < 0) {
                printf("Erro ao ler os dados\n");
            } else {
                printf("Total de bytes recebidos: %d\n", totalBytesRead);
                printf("Dados salvos no arquivo 'penguin-received.gif'\n");
            }

            llclose(1);
            return;
        }
    } else {
        printf("Função de transmissão inválida: %s\n", role);
        return;
    }
}