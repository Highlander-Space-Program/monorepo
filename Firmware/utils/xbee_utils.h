#define START_DELIMITER 0x7E
#define TRANSMIT_REQUEST 0x10

using namespace std;

uint8_t[] CreateXbeeTxPacket(uint64_t destination, uint8_t* payload, int payload_length) {
    /* Creates an XBEE TX request packet with a specified payload and destination
    
    Args:
        destination: uint64, destination address of desired XBEE.
          Can be set to 0x000000000000FFFF to broadcast to all
        payload: uint8_t array of all the data we want to send
        payload_length: int length of payload buffer
    */

    // xbee packets include 18 bytes other than payload
    int packet_length = payload_length + 18;
    uint8_t packet[packet_length];
    packet[0] = START_DELIMITER; 
}

void ParseDestination(uint64_t destination, uint8_t* packet) {
    /* Parses 64 bit destination into array of uint8_t*/
}

void CalculateChecksum(uint8_t* packet) {
    /* Calculates and inserts checksum for xbee packet*/


}

uint8_t