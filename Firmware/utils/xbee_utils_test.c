#include "xbee_utils.h"
#include <stdint.h>
#include <stdio.h>

int main(void) {
    uint64_t destination = 0x000000000000FFFF;
    uint8_t payload[] = {0x00, 0x11};
    int payload_length = 2;
    uint8_t packet[20];
    CreateXbeeTxPacket(destination, payload, payload_length, packet);

    for (int i = 0; i < 20; ++i) {
        printf("%d\n", packet[i]);
    }
}