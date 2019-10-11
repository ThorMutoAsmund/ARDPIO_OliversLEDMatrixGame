#include <Arduino.h>
#include <SPI.h>


// #include "./TimerOne.h"

#define NOP __asm__ __volatile__ ("nop\n\t")
#define PIN_CS 10
#define PIN_BTN 9
#define PIN_MOTOR 8

// MAX7219 SPI LED Driver
#define MAX7219_TEST 0x0f // in real code put into a .h file
#define MAX7219_BRIGHTNESS 0x0a // in real code put into a .h file
#define MAX7219_SCAN_LIMIT 0x0b // in real code put into a .h file
#define MAX7219_DECODE_MODE 0x09 // in real code put into a .h file
#define MAX7219_SHUTDOWN 0x0C // in real code put into a .h file

#define NUM_DISPLAYS 4

uint8_t buffer[16];

// Forwarding
void maxTransferCMD(uint8_t address, uint8_t value);
void maxTransferDATA(uint8_t address, uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4);
void transferBuffer();
void bevaegLinje();
void naesteLinje();
void faldNed(uint8_t lineStart, uint8_t blok);
void vundet();
void tabt();
void resetSpil();

// Setup
void setup()
{
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_BTN, INPUT_PULLUP);
    pinMode(PIN_MOTOR, OUTPUT);    
    digitalWrite(PIN_MOTOR, HIGH);
    
    Serial.begin(115200);
    
    SPI.setBitOrder(MSBFIRST); 
    SPI.begin();  

    maxTransferCMD(MAX7219_TEST, 0x00); // Disable text mode
    maxTransferCMD(MAX7219_DECODE_MODE, 0x00); // Disable BCD mode. 
    maxTransferCMD(MAX7219_BRIGHTNESS, 0x00);  // Use lowest intensity. 
    maxTransferCMD(MAX7219_SCAN_LIMIT, 0x0f);  // Scan all digits.
    maxTransferCMD(MAX7219_SHUTDOWN, 0x01);    // Turn on chip.
    
    // Slet display
    for (uint8_t i=1; i<=8; ++i) maxTransferDATA(i,0,0,0,0);

    // Slet buffer
    resetSpil();
}

uint8_t retning = 1;
uint8_t line = 15;
uint8_t btnPressed = 0;
uint8_t btnLongCnt;
uint8_t gameOver = 0;

void loop()
{    
    transferBuffer();
    delay(120);

    if (!gameOver)
    {
        bevaegLinje();
    }

    // Knap
    if (!btnPressed)
    {
        int sensorValue = digitalRead(PIN_BTN);
        if (!sensorValue)
        {
            btnPressed = 1;
            btnLongCnt = 0;

            if (!gameOver)
            {
                naesteLinje();

                if (line == 7)
                {
                    vundet();
                }
            }
        }
    }
    else
    {
        btnLongCnt++;

        int sensorValue = digitalRead(PIN_BTN);
        if (sensorValue)
        {
            btnPressed = 0;
        }

        if (btnLongCnt == 15)
        {
            resetSpil();
        }
    }
}

void bevaegLinje()
{
    if (retning == 1)
        buffer[line] >>= 1;
    else 
        buffer[line] <<= 1;

    if (buffer[line] & 0B00000001)
        retning = 0;
    else if (buffer[line] & 0B10000000)
        retning = 1;    
}

void naesteLinje()
{
    line--;

    if (line <= 13)
    {
        uint8_t blok = buffer[line+1] - (buffer[line+1] & buffer[line+2]);
        buffer[line+1] = buffer[line+1] & buffer[line+2];
        faldNed(line+1, blok);
        buffer[line] = buffer[line+1];
    }
    else
    {
        buffer[line] = 0B11110000;
    }

    if (!buffer[line])
    {
        tabt();
        return;
    }

    // Determine direction
    if (buffer[line] == 0B00111100 || buffer[line] == 0B00011000)
    {
        retning = line % 2; // random
    }
    else if ((buffer[line] & 0B11110000 == 0) || (buffer[line] & 0B11100000 == 0 && buffer[line] & 0B00001000 == 0B00001000))
    {
        retning = 0;
    }
    else
    {
        retning = 1;
    }

    // Byg ny linie
    uint8_t sh = buffer[line];
    if (retning)
    {
        if (!(buffer[line] & 0B11100000)) buffer[line] <<= 3;
        else if (!(buffer[line] & 0B11000000)) buffer[line] <<= 2;
        else if (!(buffer[line] & 0B10000000)) buffer[line] <<= 1;
    }
    else
    {
        if (!(buffer[line] & 0B00000111)) buffer[line] >>= 3;
        else if (!(buffer[line] & 0B00000011)) buffer[line] >>= 2;
        else if (!(buffer[line] & 0B00000001)) buffer[line] >>= 1;
    }
}

void faldNed(uint8_t lineStart, uint8_t blok)
{
    if (blok == 0)
    {
        return;
    }

    uint8_t t;
    for (uint8_t l = lineStart; l<=15; ++l)
    {
        t = buffer[l];
        buffer[l] |= blok;
        transferBuffer();
        delay(350);
        if (l<15 && (buffer[l+1] & blok))
        {
            buffer[l] = t | (buffer[l+1] & blok);
            blok -= buffer[l+1] & blok;
            if (!blok)
            {
                break;
            }
            continue;
        }
        if (l == 15)
        {
            break;
        }
        buffer[l] = t;
    }
}

void vundet()
{
    gameOver = 1;
    // Gør noget sjovt her
    // Start motor
    digitalWrite(PIN_MOTOR, LOW);
    delay(5000);
    digitalWrite(PIN_MOTOR, HIGH);
}

void tabt()
{
    gameOver = 1;
}

void resetSpil()
{
    for (uint8_t i=0; i<16; ++i) buffer[i] = 0;
    transferBuffer();

    buffer[15] = 0B11110000;

    retning = 1;
    line = 15;   
    gameOver = 0; 
}


    // Første kommer tættest på stikket
    // Tallet tættest på B'et kommer nederst
    // maxTransferDATA(1,0B00111100,0B00111111,0B01111100,0B11111111);

uint8_t bf0[8];
uint8_t bf1[8];
void transferBuffer()
{

    for (uint8_t i=0; i<8; ++i)
    {
        bf0[i] = 0;        
    }

    uint8_t y = 0B10000000;
    for (uint8_t i=0; i<8; ++i)
    {
        bf1[i] = 0;
        uint8_t x = 0B10000000;
        for (uint8_t j=15; j>=8; --j)
        {
            bf1[i] |= (buffer[j] & y ? x : 0);
            x >>= 1;
        }
        y >>= 1;
    }

    for (uint8_t i=1; i<=8; ++i)
    {
        maxTransferDATA(i,0,0,bf0[i-1],bf1[i-1]);
    }
}

void maxTransferCMD(uint8_t address, uint8_t value)
{  
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(address);      // Send address.
    SPI.transfer(value);        // Send the value.
    SPI.transfer(address);      // Send address. 
    SPI.transfer(value);        // Send the value.
    SPI.transfer(address);      // Send address. 
    SPI.transfer(value);        // Send the value.
    SPI.transfer(address);      // Send address. 
    SPI.transfer(value);        // Send the value.
    digitalWrite(PIN_CS, HIGH); // Finish transfer.
}

void maxTransferDATA(uint8_t address, uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4) 
{  
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(address);      // Send address.
    SPI.transfer(v4);        // Send the value.
    SPI.transfer(address);      // Send address. 
    SPI.transfer(v3);           // Send the value.
    SPI.transfer(address);      // Send address.
    SPI.transfer(v2);        // Send the value.
    SPI.transfer(address);      // Send address. 
    SPI.transfer(v1);           // Send the value
    digitalWrite(PIN_CS, HIGH); // Finish transfer.
}



