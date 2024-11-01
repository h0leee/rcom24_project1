#ifndef APPLICATION_LAYER_UTILS_H
#define APPLICATION_LAYER_UTILS_H
#include <cstdio>

unsigned char *getControlPacket(const unsigned int c, const char* filename, long int length, unsigned int* size);
unsigned char* parseControlPacket(unsigned char* packet, int size, unsigned long int *fileSize);
unsigned char* getDataPacket(unsigned char sequence, unsigned char *data, int dataSize, unsigned int *packetSize);
void parseDataPacket(const unsigned char* packet, int packetSize, unsigned char* buffer, int* dataLength);
unsigned char* getData(FILE* fd, long int fileLength);

#endif // APPLICATION_LAYER_UTILS_H
