// // Application layer protocol implementation

// // USAR MAX_PAYLOAD_SIZE???
// // como dar handle dos diferentes tipos de packets??????? no llread?

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include "application_layer_utils.h"
// #include "application_layer.h"
// #include "link_layer.h"

// // Constants for packet types
// #define DATA_PACKET 2
// #define START_PACKET 1
// #define END_PACKET 3

// // Control packet parameters
// #define FILE_SIZE_PARAM 0
// #define FILE_NAME_PARAM 1


// void applicationLayer(const char *serialPort, const char *role, int baudRate,
//                       int nTries, int timeout, const char *filename) {
//     // Initialize connection parameters
//     LinkLayer connectionParameters;
//     strcpy(connectionParameters.serialPort, serialPort);
//     connectionParameters.role = strcmp(role, "tx") == 0 ? LlTx : LlRx;
//     connectionParameters.baudRate = baudRate;
//     connectionParameters.nRetransmissions = nTries;
//     connectionParameters.timeout = timeout;

//     // Open the connection using link layer
//     int fd = llopen(connectionParameters);
//     if (fd < 0) {
//         fprintf(stderr, "Failed to open connection on port %s\n", serialPort);
//         exit(EXIT_FAILURE);
//     }

//     if (connectionParameters.role == LlTx) {
//         // Transmitter
//         FILE *file = fopen(filename, "rb");
//         if (file == NULL) {
//             perror("Failed to open file for reading");
//             llclose(fd);
//             exit(EXIT_FAILURE);
//         }

//         // Get file size
//         fseek(file, 0, SEEK_END);
//         long fileSize = ftell(file);
//         rewind(file);

//         // Send START packet
//         unsigned int startPacketSize;
//         unsigned char *startPacket = getControlPacket(START_PACKET, filename, fileSize, &startPacketSize);

//         if (llwrite(startPacket, startPacketSize) < 0) {
//             fprintf(stderr, "Failed to send START packet\n");
//             free(startPacket);
//             fclose(file);
//             llclose(fd);
//             exit(EXIT_FAILURE);
//         }
//         free(startPacket);

//         // Send DATA packets
//         unsigned char *fileData = getData(file, fileSize);
//         size_t bytesSent = 0;
//         int sequenceNumber = 0;

//         while (bytesSent < fileSize) {
//             size_t dataSize = (fileSize - bytesSent) > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : (fileSize - bytesSent);

//             unsigned int packetSize;
//             unsigned char *dataPacket = getDataPacket(sequenceNumber % 256, fileData + bytesSent, dataSize, &packetSize);

//             if (llwrite(dataPacket, packetSize) < 0) {
//                 fprintf(stderr, "Failed to send DATA packet\n");
//                 free(dataPacket);
//                 free(fileData);
//                 fclose(file);
//                 llclose(fd);
//                 exit(EXIT_FAILURE);
//             }

//             bytesSent += dataSize;
//             sequenceNumber++;
//             free(dataPacket);
//         }

//         free(fileData);

//         // Send END packet
//         unsigned int endPacketSize;
//         unsigned char *endPacket = getControlPacket(END_PACKET, filename, fileSize, &endPacketSize);

//         if (llwrite(endPacket, endPacketSize) < 0) {
//             fprintf(stderr, "Failed to send END packet\n");
//             free(endPacket);
//             fclose(file);
//             llclose(fd);
//             exit(EXIT_FAILURE);
//         }
//         free(endPacket);

//         fclose(file);
//     } else {
//         // Receiver
//         FILE *file = NULL;
//         int done = 0;

//         while (!done) {
//             unsigned char packet[MAX_PAYLOAD_SIZE + 4];
//             int packetSize = llread(packet);
//             if (packetSize < 0) {
//                 fprintf(stderr, "Failed to read packet\n");
//                 if (file != NULL) fclose(file);
//                 llclose(fd);
//                 exit(EXIT_FAILURE);
//             }

//             switch (packet[0]) {
//                 case START_PACKET: {
//                     // Parse START packet
//                     unsigned long fileSize = 0;
//                     unsigned char *fileName = parseControlPacket(packet, packetSize, &fileSize);

//                     file = fopen((char *)fileName, "wb");
//                     if (file == NULL) {
//                         perror("Failed to open file for writing");
//                         free(fileName);
//                         llclose(fd);
//                         exit(EXIT_FAILURE);
//                     }
//                     free(fileName);
//                     break;
//                 }
//                 case DATA_PACKET: {
//                     // Parse and write data to file
//                     int dataLength;
//                     unsigned char dataBuffer[MAX_PAYLOAD_SIZE];
//                     parseDataPacket(packet, packetSize, dataBuffer, &dataLength);
//                     fwrite(dataBuffer, 1, dataLength, file);
//                     break;
//                 }
//                 case END_PACKET: {
//                     // Finish reception
//                     done = 1;
//                     break;
//                 }
//                 default:
//                     fprintf(stderr, "Unknown packet type: %d\n", packet[0]);
//                     break;
//             }
//         }

//         if (file != NULL) fclose(file);
//     }

//     // Close the connection
//     if (llclose(fd) < 0) {
//         fprintf(stderr, "Failed to close connection\n");
//         exit(EXIT_FAILURE);
//     }
// }

// // Function to create control packets (START and END)
// unsigned char *getControlPacket(const unsigned int c, const char* filename, long int length, unsigned int* size){
//     unsigned int filenameLength = strlen(filename) + 1; // Include null terminator
//     unsigned int fileSizeLength = sizeof(long);

//     *size = 1 + 2 + fileSizeLength + 2 + filenameLength; // C + T1 + L1 + V1 + T2 + L2 + V2

//     unsigned char *packet = (unsigned char*)malloc(*size);
//     if (packet == NULL) {
//         fprintf(stderr, "Failed to allocate memory for control packet\n");
//         exit(EXIT_FAILURE);
//     }

//     unsigned int index = 0;
//     packet[index++] = c; // Control field
//     packet[index++] = FILE_SIZE_PARAM; // T1
//     packet[index++] = fileSizeLength; // L1
//     memcpy(&packet[index], &length, fileSizeLength); // V1
//     index += fileSizeLength;
//     packet[index++] = FILE_NAME_PARAM; // T2
//     packet[index++] = filenameLength; // L2
//     memcpy(&packet[index], filename, filenameLength); // V2

//     return packet;
// }

// // Function to parse control packets and extract file size and name
// unsigned char* parseControlPacket(unsigned char* packet, int size, unsigned long int *fileSize) {
//     unsigned int index = 1; // Skip control field
//     unsigned char *fileName = NULL;

//     while (index < size) {
//         unsigned char paramType = packet[index++];
//         unsigned char paramLength = packet[index++];

//         if (paramType == FILE_SIZE_PARAM) {
//             memcpy(fileSize, &packet[index], paramLength);
//         } else if (paramType == FILE_NAME_PARAM) {
//             fileName = (unsigned char*)malloc(paramLength);
//             if (fileName == NULL) {
//                 fprintf(stderr, "Failed to allocate memory for file name\n");
//                 exit(EXIT_FAILURE);
//             }
//             memcpy(fileName, &packet[index], paramLength);
//         }
//         index += paramLength;
//     }

//     return fileName;
// }

// // Function to create data packets
// unsigned char* getDataPacket(unsigned char sequence, unsigned char *data, int dataSize, unsigned int *packetSize){
//     *packetSize = 4 + dataSize; // C + N + L2 + L1 + Data
//     unsigned char *packet = (unsigned char*)malloc(*packetSize);
//     if (packet == NULL) {
//         fprintf(stderr, "Failed to allocate memory for data packet\n");
//         exit(EXIT_FAILURE);
//     }

//     packet[0] = DATA_PACKET; // Control field
//     packet[1] = sequence;    // Sequence number
//     packet[2] = dataSize / 256; // L2
//     packet[3] = dataSize % 256; // L1
//     memcpy(&packet[4], data, dataSize); // Data field

//     return packet;
// }

// // Function to parse data packets and extract the actual data
// void parseDataPacket(const unsigned char* packet, int packetSize, unsigned char* buffer, int* dataLength) {
//     // Third and fourth bytes give the data length in a two-byte field (L2, L1)
//     *dataLength = packet[2] * 256 + packet[3];

//     // Copy data from the packet to the buffer
//     memcpy(buffer, packet + 4, *dataLength);
// }

// // Function to read the entire file content into memory
// unsigned char* getData(FILE* fd, long int fileLength) {
//     unsigned char* content = (unsigned char*)malloc(fileLength);
//     if (content == NULL) {
//         fprintf(stderr, "Failed to allocate memory for file data\n");
//         exit(EXIT_FAILURE);
//     }

//     size_t bytesRead = fread(content, 1, fileLength, fd);
//     if (bytesRead != fileLength) {
//         fprintf(stderr, "Failed to read entire file into memory\n");
//         free(content);
//         exit(EXIT_FAILURE);
//     }

//     return content;
// }

