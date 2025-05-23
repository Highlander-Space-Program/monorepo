#include "main.h"
#include "W25Qxx.h"

extern SPI_HandleTypeDef hspi2;
#define W25Q_SPI hspi2

#define numBLOCK 1024

void csLOW(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

void csHIGH(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void SPI_Write(uint8_t *data, uint8_t len)
{
    HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}

void SPI_Read(uint8_t *data, uint8_t len)
{
    HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}

void W25Q_Reset(void)
{
    uint8_t tData[2];

    tData[0] = 0x66; // enable Reset
    tData[1] = 0x99; // Reset
    csLOW();
    SPI_Write(tData, 2);
    csHIGH(); // pull the HIGH
    HAL_Delay(100);
}

uint32_t W25Q_ReadID(void)
{
    uint8_t tData = 0x9f; // Read ID
    uint8_t rData[3];
    csLOW(); // pull the CS LOW
    SPI_Write(&tData, 1);
    SPI_Read(rData, 3);
    csHIGH();                                               // pull the HIGH
    return ((rData[0] << 16) | (rData[1] << 8) | rData[2]); // MFN ID : MEM ID : CAPACITY ID
}

void W25Q_Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
    uint8_t tData[5];
    uint32_t memAddr = (startPage * 256) + offset;
    if (numBLOCK < 512) // Chip Size<256Mb
    {
        tData[0] = 0x03;                   // enable Read
        tData[1] = (memAddr >> 16) & 0xFF; // MSB of the memory Address
        tData[2] = (memAddr >> 8) & 0xFF;
        tData[3] = (memAddr) & 0xFF; // LSB of the memory Address
    }
    else
    {
        tData[0] = 0x13;                   // Read Data with 4-Byte Address
        tData[1] = (memAddr >> 24) & 0xFF; // MSB of the memory Address
        tData[2] = (memAddr >> 16) & 0xFF;
        tData[3] = (memAddr >> 8) & 0xFF;
        tData[4] = (memAddr) & 0xFF; // LSB of the memory Address
    }
    csLOW(); // pull the CS Low
    if (numBLOCK < 512)
    {
        SPI_Write(tData, 4); // send read instruction along with the 24 bit memory address
    }
    else
    {
        SPI_Write(tData, 5); // send read instruction along with the 32 bit memory address
    }
    SPI_Read(rData, size); // Read the data
    csHIGH();              // pull the CS High
}

void W25Q_FastRead(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
    uint8_t tData[6];
    uint32_t memAddr = (startPage * 256) + offset;
    if (numBLOCK < 512) // Chip Size<256Mb
    {
        tData[0] = 0x0B;                   // enable Fast Read
        tData[1] = (memAddr >> 16) & 0xFF; // MSB of the memory Address
        tData[2] = (memAddr >> 8) & 0xFF;
        tData[3] = (memAddr) & 0xFF; // LSB of the memory Address
        tData[4] = 0;                // Dummy clock
    }
    else
    {
        tData[0] = 0x0C;                   // Fast Read with 4-Byte Address
        tData[1] = (memAddr >> 24) & 0xFF; // MSB of the memory Address
        tData[2] = (memAddr >> 16) & 0xFF;
        tData[3] = (memAddr >> 8) & 0xFF;
        tData[4] = (memAddr) & 0xFF; // LSB of the memory Address
        tData[5] = 0;                // Dummy clock
    }
    csLOW(); // pull the CS Low
    if (numBLOCK < 512)
    {
        SPI_Write(tData, 5); // send read instruction along with the 24 bit memory address
    }
    else
    {
        SPI_Write(tData, 6); // send read instruction along with the 32 bit memory address
    }
    SPI_Read(rData, size); // Read the data
    csHIGH();              // pull the CS High
}

void write_enable(void)
{
    uint8_t tData = 0x06; // enable Write
    csLOW();              // pull the CS LOW
    SPI_Write(&tData, 1);
    csHIGH();     // pull the HIGH
    HAL_Delay(5); // Write cycle delay (5ms)
}

void write_disable(void)
{
    uint8_t tData = 0x04; // disable Write
    csLOW();              // pull the CS LOW
    SPI_Write(&tData, 1);
    csHIGH();     // pull the HIGH
    HAL_Delay(5); // Write cycle delay (5ms)
}

void W25Q_Erase_Sector(uint16_t numsector)
{
    uint8_t tData[6];
    uint32_t memAddr = numsector * 16 * 256; // Each sector contains 16 pages * 256 bytes
    write_enable();
    if (numBLOCK < 512) // Chip Size<256Mb
    {
        tData[0] = 0x20;                   // Erase sector
        tData[1] = (memAddr >> 16) & 0xFF; // MSB of the memory Address
        tData[2] = (memAddr >> 8) & 0xFF;
        tData[3] = (memAddr) & 0xFF; // LSB of the memory Address
        csLOW();
        SPI_Write(tData, 4);
        csHIGH();
    }
    else // we use 32bit memory address for chips >= 256Mb
    {
        tData[0] = 0x21; // ERASE Sector with 32bit address
        tData[1] = (memAddr >> 24) & 0xFF;
        tData[2] = (memAddr >> 16) & 0xFF;
        tData[3] = (memAddr >> 8) & 0xFF;
        tData[4] = memAddr & 0xFF;
        csLOW(); // pull the CS LOW
        SPI_Write(tData, 5);
        csHIGH(); // pull the HIGH
    }
    HAL_Delay(450); // 450ms delay for sector erase
    write_disable();
}

uint32_t bytestowrite(uint32_t size, uint16_t offset)
{
    if ((size + offset) < 256)
        return size;
    else
        return 256 - offset;
}

void W25Q_Write_Page(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
    uint8_t tData[266];
    uint32_t startPage = page;
    uint32_t endPage = startPage + ((size + offset - 1) / 256);
    uint32_t numPages = endPage - startPage + 1;
    uint16_t startSector = startPage / 16;
    uint16_t endSector = endPage / 16;
    uint16_t numSectors = endSector - startSector + 1;
    for (uint16_t i = 0; i < numSectors; i++)
    {
        W25Q_Erase_Sector(startSector++);
    }
    uint32_t dataPosition = 0;
    // write the data
    for (uint32_t i = 0; i < numPages; i++)
    {
        uint32_t memAddr = (startPage * 256) + offset;
        uint16_t bytesremaining = bytestowrite(size, offset);
        write_enable();
        uint32_t indx = 0;
        if (numBLOCK < 512) // Chip Size<256Mb
        {
            tData[0] = 0x02;                   // page program
            tData[1] = (memAddr >> 16) & 0xFF; // MSB of the memory Address
            tData[2] = (memAddr >> 8) & 0xFF;
            tData[3] = (memAddr) & 0xFF; // LSB of the memory Address
            indx = 4;
        }
        else
        {
            tData[0] = 0x12;                   // page program with 4-Byte Address
            tData[1] = (memAddr >> 24) & 0xFF; // MSB of the memory Address
            tData[2] = (memAddr >> 16) & 0xFF;
            tData[3] = (memAddr >> 8) & 0xFF;
            tData[4] = (memAddr) & 0xFF; // LSB of the memory Address
            indx = 5;
        }
        uint16_t bytestosend = bytesremaining + indx;
        for (uint16_t i = 0; i < bytesremaining; i++)
        {
            tData[indx++] = data[i + dataPosition];
        }
        csLOW();
        SPI_Write(tData, bytestosend);
        csHIGH();
        startPage++;
        offset = 0;
        size = size - bytesremaining;
        dataPosition = dataPosition + bytesremaining;
        HAL_Delay(5);
        write_disable();
    }
}

void W25Q_Write(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
    uint16_t startSector = page / 16;
    uint16_t endSector = (page + ((size + offset - 1) / 256)) / 16;
    uint16_t numSectors = endSector - startSector + 1;
    uint8_t previousData[4096];
    uint32_t sectorOffset = ((page % 16) * 256) + offset;
    uint32_t dataindx = 0;
    for (uint16_t i = 0; i < numSectors; i++)
    {
        uint32_t startPage = startSector * 16;
        W25Q_FastRead(startPage, 0, 4096, previousData);
        uint16_t bytesRemaining = bytestowrite(size, sectorOffset);
        for (uint16_t i = 0; i < bytesRemaining; i++)
        {
            previousData[i + sectorOffset] = data[i + dataindx];
        }
        W25Q_Write_Page(startPage, 0, 4096, previousData);
        startSector++;
        sectorOffset = 0;
        dataindx = dataindx + bytesRemaining;
        size = size - bytesRemaining;
    }
}

/*
void W25Q_Write_Byte (uint32_t Addr, uint8_t data)
{
uint8_t tData[6];
uint8_t indx;
if (numBLOCK<512)   // Chip Size<256Mb
{
tData[0] = 0x02;  // page program
tData[1] = (Addr>>16)&0xFF;  // MSB of the memory Address
tData[2] = (Addr>>8)&0xFF;
tData[3] = (Addr)&0xFF; // LSB of the memory Address
tData[4] = data;
indx = 5;
}
else  // we use 32bit memory address for chips >= 256Mb
{
tData[0] = 0x12;  // Write Data with 4-Byte Address
tData[1] = (Addr>>24)&0xFF;  // MSB of the memory Address
tData[2] = (Addr>>16)&0xFF;
tData[3] = (Addr>>8)&0xFF;
tData[4] = (Addr)&0xFF; // LSB of the memory Address
tData[5] = data;
indx = 6;
}
if (W25Q_Read_Byte(Addr) == 0xFF)
{
write_enable();
csLOW();
SPI_Write(tData, indx);
csHIGH();
W25Q_Delay(5);
write_disable();
}
}
*/

void float2Bytes(uint8_t *ftoa_bytes_temp, float float_variable)
{
    union
    {
        float a;
        uint8_t bytes[4];
    } thing;
    thing.a = float_variable;
    for (uint8_t i = 0; i < 4; i++)
    {
        ftoa_bytes_temp[i] = thing.bytes[i];
    }
}

float Bytes2float(uint8_t *ftoa_bytes_temp)
{
    union
    {
        float a;
        uint8_t bytes[4];
    } thing;
    for (uint8_t i = 0; i < 4; i++)
    {
        thing.bytes[i] = ftoa_bytes_temp[i];
    }
    float float_variable = thing.a;
    return float_variable;
}

void W25Q_Write_NUM(uint32_t page, uint16_t offset, float data)
{
    uint8_t tempBytes[4];
    float2Bytes(tempBytes, data);
    /* Write using sector update function */
    W25Q_Write(page, offset, 4, tempBytes);
}

float W25Q_Read_NUM(uint32_t page, uint16_t offset)
{
    uint8_t rData[4];
    W25Q_Read(page, offset, 4, rData);
    return (Bytes2float(rData));
}

void W25Q_Write_32B(uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
    uint8_t data8[size * 4];
    uint32_t indx = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        data8[indx++] = data[i] & 0xFF; // extract LSB
        data8[indx++] = (data[i] >> 8) & 0xFF;
        data8[indx++] = (data[i] >> 16) & 0xFF;
        data8[indx++] = (data[i] >> 24) & 0xFF;
    }
    W25Q_Write(page, offset, indx, data8);
}

void W25Q_Read_32B(uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
    uint8_t data8[size * 4];
    uint32_t indx = 0;
    W25Q_FastRead(page, offset, size * 4, data8);
    for (uint32_t i = 0; i < size; i++)
    {
        data[i] = (data8[indx++]) | (data8[indx++] << 8) | (data8[indx++] << 16) | (data8[indx++] << 24);
    }
}
