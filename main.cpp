#include <iostream>
#include "LoRa.h"

using namespace std;

int button1 = 22;
int button2 = 23;
int led1 = 24;
int led2 = 25;

void button1_ISR(void);
void button2_ISR(void);

int main(int argc, char **argv)
{
    pinMode(led1, OUTPUT);
    digitalWrite(led1, HIGH);
    pinMode(led2, OUTPUT);
    digitalWrite(led2, HIGH);

    pinMode(button1, INPUT);
    pullUpDnControl(button1, PUD_UP);
    wiringPiISR(button1, INT_EDGE_FALLING, &button1_ISR);

    pinMode(button2, INPUT);
    pullUpDnControl(button2, PUD_UP);
    wiringPiISR(button2, INT_EDGE_FALLING, &button2_ISR);

    LoRa.setPins(2, 0, 7);
    uint8_t res = LoRa.begin(868500000);
    if (res != 1)
    {
        cout << "Starting LoRa failed: " << (int)res << endl;
        // exit(1);
        while (true)
            ;
    }

    LoRa.setSignalBandwidth(500000);
    LoRa.setCodingRate4(5);
    LoRa.setSpreadingFactor(12);
    LoRa.setPreambleLength(8);

    cout << "Initialize done, listenning ..." << endl;

    while (true)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            cout << "Received packet '";

            // read packet
            while (LoRa.available())
            {
                cout << (char)LoRa.read();
            }

            // print RSSI of packet
            cout << "' with RSSI ";
            cout << LoRa.packetRssi() << endl;
        }
    }

    return 0;
}

void button1_ISR(void)
{
    digitalWrite(led1, HIGH);
}

void button2_ISR(void)
{
    digitalWrite(led1, LOW);
}
