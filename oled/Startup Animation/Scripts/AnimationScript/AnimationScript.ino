#include <Wire.h>
#include <U8g2lib.h>
#include "frame1.h"
#include "frame2.h"
#include "frame3.h"
#include "frame4.h"
#include "frame5.h"
#include "frame6.h"
#include "frame7.h"
#include "frame8.h"
#include "frame9.h"
#include "frame10.h"
#include "frame11.h"
#include "frame12.h"

// Initialize SH1106 OLED with I2C address 0x60
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, 0x60);

// Array to hold all frames
const unsigned char* frames[] = {
    myBitmapSprite_0001, myBitmapSprite_0002, myBitmapSprite_0003, 
    myBitmapSprite_0004, myBitmapSprite_0005, myBitmapSprite_0006, 
    myBitmapSprite_0007, myBitmapSprite_0008, myBitmapSprite_0009, 
    myBitmapSprite_0010, myBitmapSprite_0011, myBitmapSprite_0012
};

const int frameCount = 12;
const int maxLoops = 5;
int loopCounter = 0;

void setup() {
    u8g2.setI2CAddress(0x60 * 2);  // Set correct I2C address
    u8g2.begin();
}

void loop() {
    if (loopCounter < maxLoops) {
        for (int i = 0; i < frameCount; i++) {
            u8g2.clearBuffer();
            u8g2.drawXBMP(0, 0, 128, 64, frames[i]);
            u8g2.sendBuffer();
            delay(100);
        }
        loopCounter++;
    } else {
        u8g2.clearBuffer();
        u8g2.drawXBMP(0, 0, 128, 64, frames[frameCount - 1]);
        u8g2.sendBuffer();
        while (1);
    }
}
