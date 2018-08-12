#include "LoRa.h"

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42

// modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

#define MAX_PKT_LENGTH 255

#define bitWrite(x, n, b) (x = (x & ~(1 << n)) | (b << n))

LoRaClass::LoRaClass() : _spiFd(-1), _ss(-1), _reset(-1), _dio0(-1), _frequency(0), _packetIndex(0), _implicitHeaderMode(0), _onReceive(NULL)
{
    wiringPiSetup();
}

int LoRaClass::begin(long frequency)
{
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, HIGH);
    usleep(200000);
    digitalWrite(_reset, LOW);
    usleep(200000);
    digitalWrite(_reset, HIGH);
    usleep(50000);

    // setup pins
    pinMode(_ss, OUTPUT);
    // set SS high
    digitalWrite(_ss, HIGH);

    if (_reset != -1)
    {
        pinMode(_reset, OUTPUT);

        // perform reset
        digitalWrite(_reset, LOW);
        usleep(10000);
        digitalWrite(_reset, HIGH);
        usleep(10000);
    }

    // start SPI
    _spiFd = wiringPiSPISetup(SPI_CHANNEL, 500000);
    usleep(100000);
    
    // check version
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12)
    {
        return version;
    }

    // put in sleep mode
    sleep();

    // set frequency
    setFrequency(frequency);

    // set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // set auto AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    setTxPower(17);

    // put in standby mode
    idle();

    return 1;
}

void LoRaClass::end()
{
    // put in sleep mode
    sleep();

    // stop SPI
}

void LoRaClass::setTxPower(int level, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin)
    {
        // RFO
        if (level < 0)
        {
            level = 0;
        }
        else if (level > 14)
        {
            level = 14;
        }

        writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else
    {
        // PA BOOST
        if (level < 2)
        {
            level = 2;
        }
        else if (level > 17)
        {
            level = 17;
        }

        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRaClass::setFrequency(long frequency)
{
    _frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRaClass::setSpreadingFactor(int sf)
{
    if (sf < 6)
    {
        sf = 6;
    }
    else if (sf > 12)
    {
        sf = 12;
    }

    if (sf == 6)
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    setLdoFlag();
}

void LoRaClass::setSignalBandwidth(long sbw)
{
    int bw;

    if (sbw <= 7.8E3)
    {
        bw = 0;
    }
    else if (sbw <= 10.4E3)
    {
        bw = 1;
    }
    else if (sbw <= 15.6E3)
    {
        bw = 2;
    }
    else if (sbw <= 20.8E3)
    {
        bw = 3;
    }
    else if (sbw <= 31.25E3)
    {
        bw = 4;
    }
    else if (sbw <= 41.7E3)
    {
        bw = 5;
    }
    else if (sbw <= 62.5E3)
    {
        bw = 6;
    }
    else if (sbw <= 125E3)
    {
        bw = 7;
    }
    else if (sbw <= 250E3)
    {
        bw = 8;
    }
    else /*if (sbw <= 250E3)*/
    {
        bw = 9;
    }

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    setLdoFlag();
}

void LoRaClass::setCodingRate4(int denominator)
{
    if (denominator < 5)
    {
        denominator = 5;
    }
    else if (denominator > 8)
    {
        denominator = 8;
    }

    int cr = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(long length)
{
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSyncWord(int sw)
{
    writeRegister(REG_SYNC_WORD, sw);
}

void LoRaClass::setPins(int ss, int reset, int dio0)
{
    _ss = ss;
    _reset = reset;
    _dio0 = dio0;
}

int LoRaClass::parsePacket(int size)
{
    int packetLength = 0;
    int irqFlags = readRegister(REG_IRQ_FLAGS);

    if (size > 0)
    {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
    {
        explicitHeaderMode();
    }

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        // received a packet
        _packetIndex = 0;

        // read packet length
        if (_implicitHeaderMode)
        {
            packetLength = readRegister(REG_PAYLOAD_LENGTH);
        }
        else
        {
            packetLength = readRegister(REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        idle();
    }
    else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
        // not currently in RX mode

        // reset FIFO address
        writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int LoRaClass::available()
{
    return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
    if (!available())
    {
        return -1;
    }

    _packetIndex++;

    return readRegister(REG_FIFO);
}

int LoRaClass::packetRssi()
{
    return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

void LoRaClass::idle()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

uint8_t LoRaClass::readRegister(uint8_t address)
{
    return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
    singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
    unsigned char buffer[2];

    memset((void*)buffer, 0xff, 2);
    buffer[0] = address;
    buffer[1] = value;

    digitalWrite(_ss, LOW);

    wiringPiSPIDataRW(SPI_CHANNEL, buffer, 2);

    digitalWrite(_ss, HIGH);

    return buffer[1];
}

void LoRaClass::setLdoFlag()
{
    // Section 4.1.1.5
    long symbolDuration = 1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()));

    // Section 4.1.1.6
    uint8_t ldoOn = 0;
    if (symbolDuration > 16)
    {
        ldoOn = 1;
    }

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRaClass::explicitHeaderMode()
{
    _implicitHeaderMode = 0;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
    _implicitHeaderMode = 1;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

int LoRaClass::getSpreadingFactor()
{
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

long LoRaClass::getSignalBandwidth()
{
    uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);
    switch (bw)
    {
    case 0:
        return 7.8E3;
    case 1:
        return 10.4E3;
    case 2:
        return 15.6E3;
    case 3:
        return 20.8E3;
    case 4:
        return 31.25E3;
    case 5:
        return 41.7E3;
    case 6:
        return 62.5E3;
    case 7:
        return 125E3;
    case 8:
        return 250E3;
    case 9:
        return 500E3;
    default:
        return 0;
    }
}

LoRaClass LoRa;
