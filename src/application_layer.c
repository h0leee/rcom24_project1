// Application layer protocol implementation.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "application_layer.h"
#include "link_layer.h"

// Constants for packet types
#define DATA_PACKET 1
#define START_PACKET 2
#define END_PACKET 3

// Maximum size of data field in a packet
#define MAX_DATA_SIZE 1000

// Control packet parameters
#define FILE_SIZE_PARAM 0
#define FILE_NAME_PARAM 1

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    // Open the connection using link layer
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = strcmp(role, "tx") == 0 ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) < 0) {
        fprintf(stderr, "Failed to open connection on port %s\n", serialPort);
        exit(EXIT_FAILURE);
    }

    if (connectionParameters.role == LlTx) {
        // Transmitter
        FILE *file = fopen(filename, "rb");
        if (file == NULL) {
            perror("Failed to open file for reading");
            llclose(0);
            exit(EXIT_FAILURE);
        }

        // Get file size
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        rewind(file);

        // Send START packet
        unsigned char startPacket[256];
        int startPacketSize = 0;
        startPacket[startPacketSize++] = START_PACKET;
        startPacket[startPacketSize++] = FILE_SIZE_PARAM;
        startPacket[startPacketSize++] = sizeof(long);
        memcpy(&startPacket[startPacketSize], &fileSize, sizeof(long));
        startPacketSize += sizeof(long);
        startPacket[startPacketSize++] = FILE_NAME_PARAM;
        int fileNameLength = strlen(filename) + 1;
        startPacket[startPacketSize++] = fileNameLength;
        memcpy(&startPacket[startPacketSize], filename, fileNameLength);
        startPacketSize += fileNameLength;

        if (llwrite(startPacket, startPacketSize) < 0) {
            fprintf(stderr, "Failed to send START packet\n");
            fclose(file);
            llclose(0);
            exit(EXIT_FAILURE);
        }

        // Send DATA packets
        unsigned char dataPacket[MAX_DATA_SIZE + 4];
        int sequenceNumber = 0;
        size_t bytesRead;
        while ((bytesRead = fread(&dataPacket[4], 1, MAX_DATA_SIZE, file)) > 0) {
            dataPacket[0] = DATA_PACKET;
            dataPacket[1] = sequenceNumber % 256;
            dataPacket[2] = bytesRead / 256;
            dataPacket[3] = bytesRead % 256;

            if (llwrite(dataPacket, bytesRead + 4) < 0) {
                fprintf(stderr, "Failed to send DATA packet\n");
                fclose(file);
                llclose(0);
                exit(EXIT_FAILURE);
            }
            sequenceNumber++;
        }

        // Send END packet (same as START packet)
        if (llwrite(startPacket, startPacketSize) < 0) {
            fprintf(stderr, "Failed to send END packet\n");
            fclose(file);
            llclose(0);
            exit(EXIT_FAILURE);
        }

        fclose(file);
    } else {
        // Receiver
        FILE *file = NULL;
        unsigned char packet[MAX_DATA_SIZE + 4];
        int done = 0;

        while (!done) {
            int packetSize = llread(packet);
            if (packetSize < 0) {
                fprintf(stderr, "Failed to read packet\n");
                if (file != NULL) fclose(file);
                llclose(0);
                exit(EXIT_FAILURE);
            }

            switch (packet[0]) {
                case START_PACKET: {
                    // Parse START packet
                    int index = 1;
                    long fileSize = 0;
                    char fileName[256] = {0};

                    while (index < packetSize) {
                        unsigned char paramType = packet[index++];
                        unsigned char paramLength = packet[index++];
                        if (paramType == FILE_SIZE_PARAM) {
                            memcpy(&fileSize, &packet[index], paramLength);
                        } else if (paramType == FILE_NAME_PARAM) {
                            memcpy(fileName, &packet[index], paramLength);
                        }
                        index += paramLength;
                    }

                    file = fopen(fileName, "wb");
                    if (file == NULL) {
                        perror("Failed to open file for writing");
                        llclose(0);
                        exit(EXIT_FAILURE);
                    }
                    break;
                }
                case DATA_PACKET: {
                    // Write data to file
                    int sequenceNumber = packet[1];
                    int dataLength = packet[2] * 256 + packet[3];
                    fwrite(&packet[4], 1, dataLength, file);
                    break;
                }
                case END_PACKET: {
                    // Finish reception
                    done = 1;
                    break;
                }
                default:
                    fprintf(stderr, "Unknown packet type: %d\n", packet[0]);
                    break;
            }
        }

        if (file != NULL) fclose(file);
    }

    // Close the connection
    if (llclose(1) < 0) {
        fprintf(stderr, "Failed to close connection\n");
        exit(EXIT_FAILURE);
    }
}
