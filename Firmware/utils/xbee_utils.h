#ifndef __XBEE_UTILS_H
#define __XBEE_UTILS_H

#include <stdint.h>
#include <stdio.h>

#define START_DELIMITER 0x7E
#define TRANSMIT_REQUEST 0x10

/* API to be able to send TX and RX packets to XBEE radios.
More info on XBEE packets can be found at:
https://www.digi.com/resources/documentation/DigiDocs/90002002/Default.htm#Containers/cont_frame_descriptions.htm?TocPath=Frame%2520descriptions%257C_____0
*/

void ConfigurePacket(uint8_t* packet, int packet_length) {
    /* Configures bytes common to all XBEE TX packets*/

    // set start byte
    packet[0] = START_DELIMITER;

    // set length bytes
    uint16_t frame_length = packet_length - 4;
    uint8_t frame_length_MSB = (uint8_t)(frame_length >> 8);
    uint8_t frame_length_LSB = (uint8_t)(frame_length & 0xFF);
    packet[1] = frame_length_MSB;
    packet[2] = frame_length_LSB;

    // set frame type
    packet[3] = 0x00;

    //set 16 bit address
    packet[12] = 0x00;
    packet[13] = 0x00;

    // set broadcast radius
    packet[14] = 0x00;

    //set transmit options
    packet[15] = 0x00;
}

void ParseDestination(uint64_t destination, uint8_t* packet) {
    /* Inserts destination into packet */

    // adddress starts at 4 with MSB
    for (int i = 0; i < 8; ++i) {
        packet[i + 4] = (uint8_t)(destination >> (56 - (8 * i)) & 0xFF);
    }
}

uint8_t* CreateXbeeTxPacket(uint64_t destination, uint8_t* payload, int payload_length, uint8_t* packet) {
    /* Creates an XBEE TX request packet with a specified payload and destination and puts it in packet
    
    Args:
        destination: uint64, destination address of desired XBEE.
          Can be set to 0x000000000000FFFF to broadcast to all
        payload: uint8_t array of all the data we want to send
        payload_length: int length of payload buffer
        packet: uint8_t*, empty array with length of packet.
          To get this length just add 18 to payload length.
    */

    // xbee packets include 18 bytes other than payload
    uint16_t packet_length = payload_length + 18;
    ParseDestination(destination, packet);
    ConfigurePacket(packet, packet_length);

    // set payload bytes
    for (int i = 16; i < payload_length + 16; ++i) {
        packet[i] = payload[i - 16];
    }

    //  set checksum
    uint8_t checksum = 0;
    for (int i = 3; i < packet_length - 1; ++i) {
        checksum += packet[i];
    }
    checksum = 0xFF - checksum;
    packet[packet_length - 1] = checksum;
}

void ParseXbeeRxPacket(uint8_t* packet, int packet_length, uint8_t* buffer) {
    /* Extracts sender address and data from xbee packet and puts it in buffer
    
    Args:
        packet: uint8_t*, raw xbee packet
        packet_length: int, length of the packet
        buffer: uint8_t*, array with length of return.
          To get length of this subtract 10 from packet length
    */

    // extracts 64 bit address that starts at index 4 in packet
    for (int i = 0; i < 8; ++i) {
        buffer[i] = packet[i + 4];
    }
    
    // extracts payload that starts at index 16 and ends at second to last byte
    for (int i = 16; i < packet_length - 1; ++i) {
        buffer[i - 11] = packet[i];
    }
}

#endif