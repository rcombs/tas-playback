/*
 * Copyright (c) 2009 Andrew Brown
 * Copyright (c) 2018 Rodger Combs
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "crc_table.h"
#include <SdFatConfig.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <BlockDriver.h>
#include <SysCall.h>

#define SD_SS_PIN 4
#define STATUS_PIN 13
#define CS 10
#define DC 9
#define RESET 7 // Moved from 8, which is in use by the N64

#define NEXT 2
#define SELECT 3

#define SERIAL_BAUD_RATE 115200

#define N64_PIN 2

#define N64_HIGH (CORE_PIN2_DDRREG &= ~CORE_PIN2_BITMASK) //digitalWriteFast(N64_PIN, HIGH)
#define N64_LOW (CORE_PIN2_DDRREG |= CORE_PIN2_BITMASK) //digitalWriteFast(N64_PIN, LOW)
#define N64_QUERY ((CORE_PIN2_PINREG & CORE_PIN2_BITMASK) ? 1 : 0) //digitalReadFast(N64_PIN)

#define LED_HIGH (CORE_PIN13_PORTSET = CORE_PIN13_BITMASK) //digitalWriteFast(STATUS_PIN, HIGH)
#define LED_LOW (CORE_PIN13_PORTCLEAR = CORE_PIN13_BITMASK) //digitalWriteFast(STATUS_PIN, LOW)

#define INPUT_BUFFER_SIZE 512 // This is ideal since it allows us to read 256*4/2 = 512 bytes
// at once. 512 bytes is an optimization for reading the sd card and skips using another buffer.

#define INPUT_BUFFER_UPDATE_TIMEOUT 10 // 10 ms

static char n64_raw_dump[36]; // maximum recv is 1+2+32 bytes + 1 bit
// n64_raw_dump does not include the command byte. That gets pushed into
// n64_command:
static unsigned char n64_command;
// bytes to send to the 64
// maximum we'll need to send is 33, 32 for a read request and 1 CRC byte
static unsigned char n64_buffer[33];
static void get_n64_command();
static void n64_send(unsigned char *buffer, char length, bool wide_stop);

// Simple switch buffer. (If buffer A fails to load while buffer B is in use,
// we still okay, and will try again next loop)
static unsigned long inputBuffer[INPUT_BUFFER_SIZE];
static bool bufferALoaded, bufferBLoaded;
static bool bufferAInUse, bufferBInUse;
static long bufferEndPos;
static bool bufferOneMore;
static long bufferPos;
static void updateInputBuffer();


static FatFile m64File;
static bool m64OpenSuccess = false;
static bool openM64(const String& path);

static bool finished = false;

//static unsigned int progressPos = 0;
static unsigned long numFrames = 0, curFrame = 0;

SdFatSdioEX sd;

#define dirPos curFrame
#define numFiles numFrames

#define MICRO_CYCLES (F_CPU / 1000000)

static void emitList(const String& path)
{
  FatFile dir;
  if (!dir.open(&sd, path.c_str(), O_READ)) {
    Serial.println(F("Failed to open requested directory"));
    return;
  }
  dir.rewind();
  while (m64File.openNext(&dir)) {
    char name[256];
    m64File.getName(name, sizeof(name));
    Serial.print("A:");
    Serial.println(name);
    m64File.close();
  }
  dir.close();
}

void setup()
{
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) { ; } // wait for serial port to connect. Needed for native USB port only

  // Status LED
  digitalWrite(STATUS_PIN, LOW);
  pinMode(STATUS_PIN, OUTPUT);

  // Communication with the N64 on this pin
  digitalWrite(N64_PIN, LOW);
  pinMode(N64_PIN, INPUT_PULLUP);

  attachInterrupt(N64_PIN, n64Interrupt, FALLING);

  // Let the N64 line interrupt anything else
  NVIC_SET_PRIORITY(IRQ_PORTD, 1);

  // Initialize SD card
  if (!sd.begin()) {
    m64OpenSuccess = false; 
    Serial.println(F("SD initialization failed!"));
    return;
  }
  Serial.println(F("SD initialization done."));

  // Setup buffer
  bufferALoaded = false;
  bufferBLoaded = false;
  bufferAInUse = false;
  bufferBInUse = false;
  bufferEndPos = -1;
  bufferOneMore = true;
  bufferPos = -1;

  Serial.println(F("Initialization done."));
}

void logFrame()
{
  const unsigned char *dat = (const unsigned char*)(inputBuffer + bufferPos);
  Serial.write("F:");
  Serial.print(curFrame);
  Serial.write(" ");
  Serial.print(dat[0], HEX);
  Serial.write(" ");
  Serial.print(dat[1], HEX);
  Serial.write(" ");
  Serial.print(dat[2], HEX);
  Serial.write(" ");
  Serial.println(dat[3], HEX);
}

static FatFile writeFile;

const bool skippingInput = false;
static String inputString;

static void handleCommand(const String& cmd)
{
  if (cmd.startsWith("M:")) {
    openM64(cmd.substring(2));
  } else if (cmd.startsWith("O:")) {
    //dummy
  } else if (cmd.startsWith("L:")) {
    emitList(cmd.substring(2));
  } else if (cmd.startsWith("MK:")) {
    Serial.print("MK:");
    Serial.println(sd.mkdir(cmd.substring(3).c_str()));
  } else if (cmd.startsWith("CR:")) {
    if (writeFile.isOpen()) {
      writeFile.close();
    }
    if (writeFile.open(&sd, cmd.substring(3).c_str(), O_WRITE | O_CREAT)) {
      Serial.println("CR:OK");
    } else {
      Serial.println("CR:NAK");
    }
  } else if (cmd.startsWith("AP:")) {

  } else if (cmd.startsWith("CL:")) {
    writeFile.close();
  } else {
    Serial.print("Unknown CMD:");
    Serial.println(cmd);
  }
}

static void inputLoop()
{
  int newChar;
  int charsRead = 0;
  while ((newChar = Serial.read()) != -1 && (charsRead++ <= 1024)) {
    if (newChar == '\n') {
      if (skippingInput)
        Serial.println("Skipped line longer than 256 chars");
      else
        handleCommand();
      inputString = "";
      skippingInput = false;
    } else if (inputString.length() > 256) {
      skippingInput = true;
    } else {
      inputString += newChar;
    }
  }
}

static void mainLoop()
{
  updateInputBuffer();
  
  // Record if it took longer than expected
  /*updateTime = readTimer();
  if (updateTime > 1000 * MICRO_CYCLES) {
      Serial.print(F("Input buffer update took too long ("));
      Serial.print(updateTime / MICRO_CYCLES);
      Serial.println(F(" us)"));
  }*/
}

static void n64Interrupt()
{
    long pos;
    unsigned char data, addr;
    startTimer();
    volatile uint32_t *config = portConfigRegister(N64_PIN);
    uint32_t oldConfig = *config;
//    unsigned int curPos;

    // Bail if file open failed
    if (!m64OpenSuccess)
        return;

    // wait to make sure the line is idle before
    // we begin listening

    noInterrupts();

    *config &= ~0x000F0000;

    // Wait for incoming 64 command
    // this will block until the N64 sends us a command
    get_n64_command();

    // 0x00 is identify command
    // 0x01 is status
    // 0x02 is read
    // 0x03 is write
    switch (n64_command)
    {
        case 0x00:
        case 0xFF:
            // identify
            // mutilate the n64_buffer array with our status
            // we return 0x050001 to indicate we have a rumble pack
            // or 0x050002 to indicate the expansion slot is empty
            //
            // 0xFF I've seen sent from Mario 64 and Shadows of the Empire.
            // I don't know why it's different, but the controllers seem to
            // send a set of status bytes afterwards the same as 0x00, and
            // it won't work without it.
            n64_buffer[0] = 0x05;
            n64_buffer[1] = 0x00;
            n64_buffer[2] = 0x01;

            n64_send(n64_buffer, 3, 1);
            Serial.println(F("Controller identified"));
            Serial.println("P:");
            break;
        case 0x01:
            // If the TAS is finished, there's nothing left to do.
            if (finished && !bufferOneMore)
              memset(n64_buffer, 0, 4);
            else
              memcpy(n64_buffer, inputBuffer + bufferPos, 4);
        
            // blast out the pre-assembled array in n64_buffer
            n64_send(n64_buffer, 4, 1);

            if (finished)
              break;

            logFrame();

            // update input buffer and make sure it doesn't take too long

           /*Serial.print(F("Pos: "));
            Serial.print(bufferPos);
            Serial.print(F(" Data: "));
            Serial.println(inputBuffer[bufferPos]);*/
            
            if (curFrame == numFrames && !finished) {
              Serial.println(F("TAS finished playing"));
              Serial.println("C:");
              finished = true;
            }

            curFrame++;

            pos = bufferPos + 1;
            if (pos >= INPUT_BUFFER_SIZE) 
                pos = 0;
            bufferPos = pos;

//            while (Serial.availableForWrite() && readTimer() < 10 * 1000 * MICRO_CYCLES && N64_QUERY);

/*            if (!finished)
              curPos = (curFrame * screen.width()) / numFrames;
            screen.fill(255,0,0);
            while (progressPos < curPos) {
              screen.point(progressPos++, screen.height() - 1);
            }
            screen.fill(255,255,255);*/

            break;
        case 0x02:
            // A read. If the address is 0x8000, return 32 bytes of 0x80 bytes,
            // and a CRC byte.  this tells the system our attached controller
            // pack is a rumble pack

            // Assume it's a read for 0x8000, which is the only thing it should
            // be requesting anyways
            memset(n64_buffer, 0x80, 32);
            n64_buffer[32] = 0xB8; // CRC

            n64_send(n64_buffer, 33, 1);
            Serial.println(F("Got a read, what?"));

            //Serial.println("It was 0x02: the read command");
            break;
        case 0x03:
            // A write. we at least need to respond with a single CRC byte.  If
            // the write was to address 0xC000 and the data was 0x01, turn on
            // rumble! All other write addresses are ignored. (but we still
            // need to return a CRC)

            // decode the first data byte (fourth overall byte), bits indexed
            // at 24 through 31
            data = 0;
            data |= (n64_raw_dump[16] != 0) << 7;
            data |= (n64_raw_dump[17] != 0) << 6;
            data |= (n64_raw_dump[18] != 0) << 5;
            data |= (n64_raw_dump[19] != 0) << 4;
            data |= (n64_raw_dump[20] != 0) << 3;
            data |= (n64_raw_dump[21] != 0) << 2;
            data |= (n64_raw_dump[22] != 0) << 1;
            data |= (n64_raw_dump[23] != 0);

            // get crc byte, invert it, as per the protocol for
            // having a memory card attached
            n64_buffer[0] = crc_repeating_table[data] ^ 0xFF;

            // send it
            n64_send(n64_buffer, 1, 1);
            Serial.println(F("Got a write, what?"));

            // end of time critical code
            // was the address the rumble latch at 0xC000?
            // decode the first half of the address, bits
            // 8 through 15
            addr = 0;
            addr |= (n64_raw_dump[0] != 0) << 7;
            addr |= (n64_raw_dump[1] != 0) << 6;
            addr |= (n64_raw_dump[2] != 0) << 5;
            addr |= (n64_raw_dump[3] != 0) << 4;
            addr |= (n64_raw_dump[4] != 0) << 3;
            addr |= (n64_raw_dump[5] != 0) << 2;
            addr |= (n64_raw_dump[6] != 0) << 1;
            addr |= (n64_raw_dump[7] != 0);

            //Serial.println("It was 0x03: the write command");
            //Serial.print("Addr was 0x");
            //Serial.print(addr, HEX);
            //Serial.print(" and data was 0x");
            //Serial.println(data, HEX);
            break;

        default:
            Serial.print(F("Unknown command: 0x"));
            Serial.println(n64_command, HEX);
            break;
    }

    *config = oldConfig;

    interrupts();
}


void loop()
{
    inputLoop();
    mainLoop();
}

static bool openM64(const String& path) {
    char signature[4];
    int version;

    Serial.write("M:");
    Serial.println(path);
  
    // Open the file for reading:
    Serial.print(F("Opening file '"));
    Serial.print(path);
    Serial.println(F("'..."));

    Serial.flush();

    if (m64File.isOpen())
      m64File.close();
  
    // Error check
    if (!m64File.open(&sd, path.c_str(), O_READ)) {
        Serial.println(F("Error in opening file"));
        return false;
    }
  
    // Open header
    if (m64File.read(signature, 4) != 4 || m64File.read(&version, 4) != 4) {
        m64File.close();
        Serial.println(F("Failed to read signature"));
        return false;
    }
  
    // Validate file signature
    if (memcmp(signature, "M64\x1A", 4) != 0) {
        Serial.println(F("m64 signature invalid"));
        m64File.close();
        return false;
    }
      
    // Print version
    Serial.print(F("M64 Version: "));
    Serial.println(version);

    Serial.flush();

    m64File.seekSet(0x018);
  
    // Open header
    if (m64File.read(&numFrames, 4) != 4) {
        m64File.close();
        Serial.println(F("Failed to read frame count"));
        return false;
    }
  
    // Get header size
    switch(version) {
        case 1:
        case 2:
            m64File.seekSet(0x200);
            break;
        case 3:
            m64File.seekSet(0x400);
            break;
        default:
          // Unknown version
            Serial.println(F("Error: unknown M64 version"));
            m64File.close();
            return false;
    }
  
    // Final check
    if (!m64File.available()) {
        Serial.println(F("No input data found in file"));
        m64File.close();
        return false;
    }

    Serial.println(F("File opened successfully"));

    Serial.print(F("Total size: "));
    Serial.println(m64File.fileSize());
    
    Serial.print(F("Position: "));
    Serial.println(m64File.curPosition());
    
    Serial.print(F("Expected Frames: "));
    Serial.println((m64File.fileSize() - m64File.curPosition()) / 4);

    numFrames = min(numFrames, (m64File.fileSize() - m64File.curPosition()) / 4);

    Serial.write("N:");
    Serial.println(numFrames);

    Serial.flush();

    // Wait for the line to go idle, then begin listening
    for (int idle_wait=32; idle_wait>0; --idle_wait) {
        if (!N64_QUERY) {
            idle_wait = 32;
        }
    }

    noInterrupts();
    m64OpenSuccess = true;
    finished = false;
    bufferPos = -1;
    bufferALoaded = false;
    bufferBLoaded = false;
    interrupts();
 
    return true;
}


static void updateInputBuffer() {
    if (finished)
      return;
  
    long readBytes = 0;
    if (bufferPos == -1) {
        // Initially, both of our buffers are not in use
        bufferPos = 0;
        bufferAInUse = false;
        bufferBInUse = false;
    }
    else {
        // Calculate next buffer position
        // Wrap position

        // Calculate which buffer (A or B) is in use
        if (bufferPos < INPUT_BUFFER_SIZE / 2) {
            // Verify the buffer in use is loaded
            if (!bufferALoaded) {
                Serial.println(F("Error: New input was not loaded in buffer."));
            }

            // Clear old buffer when loading
            if (!bufferAInUse) {
                bufferBLoaded = false;
            }
            
            bufferAInUse = true;
            bufferBInUse = false;
        } else {
            if (!bufferBLoaded) {
                Serial.println(F("Error: New input was not loaded in buffer."));
            }

            // Clear old buffer when loading
            if (!bufferBInUse) {
                bufferALoaded = false;
            }
            
            bufferAInUse = false;
            bufferBInUse = true;
        }
    }
    
    // If we have loaded all inputs, wrap up
    if (bufferEndPos != -1) {
        // Perform one more input then end once the last input data has been queued.
        if (bufferOneMore && bufferPos == bufferEndPos) 
            bufferOneMore = false;
            
        return;
    }
  
    // Check for file end
    if (!m64File.available()) {
//        screen.println(F("End of M64 loaded into buffer"));
        
        // Set end
        if (!bufferAInUse) {
            // End of buffer B marks end when
            bufferEndPos = INPUT_BUFFER_SIZE - 1;
        } 
        else if (!bufferBInUse) {
            // End of buffer A marks end
            bufferEndPos = INPUT_BUFFER_SIZE/2 - 1;
        } else {
            // Da fuq? (This should never happen)
            Serial.println(F("Unexpected ending of input file"));
      
            // End immediatley
            bufferEndPos = 0;
            finished = true;
        }
    
        // Close file
        m64File.close();
        return;
    }
  
    // Load buffer A when its not in use
    if (!bufferALoaded && !bufferAInUse) {
        //Serial.println(F("Loading inputs into buffer A"));
        // Read bytes
        readBytes = m64File.read(inputBuffer, INPUT_BUFFER_SIZE*2);
        if (readBytes == 0) {
            Serial.println(F("Failed to read next inputs from file. (This is recoverable)"));
        }
        else {
            bufferALoaded = true;
            if (readBytes != INPUT_BUFFER_SIZE*2) {
//                screen.println(F("End of M64 file found. Loading remaining bytes."));
                bufferEndPos = readBytes / 4 - 1;
                // Close file
                m64File.close();
            }
        }
    }
    // Load buffer B when its not in use
    else if (!bufferBLoaded && !bufferBInUse) {
        //Serial.println(F("Loading inputs into buffer B"));
        // Read bytes
        readBytes = m64File.read(inputBuffer + (INPUT_BUFFER_SIZE / 2), INPUT_BUFFER_SIZE*2);
        if (readBytes == 0) {
            Serial.println(F("Failed to read next inputs from file. (This is recoverable)"));
        }
        else {
            bufferBLoaded = true;
            if (readBytes != INPUT_BUFFER_SIZE*2) {
//                screen.println(F("End of M64 file found. Loading remaining bytes."));
                // buffer end is offset from middle
                bufferEndPos = INPUT_BUFFER_SIZE / 2 + readBytes / 4 - 1;
                // Close file
                m64File.close();
            }
        }
    }
}

static void startTimer()
{
  ARM_DWT_CYCCNT = 0;
}

static long readTimer()
{
  return ARM_DWT_CYCCNT;
}

/**
 * Complete copy and paste of gc_send, but with the N64
 * pin being manipulated instead.
 */
static void n64_send(unsigned char *buffer, char length, bool wide_stop)
{
    long target;
    LED_HIGH;
    N64_LOW;
    startTimer();
 
    for (int i = 0; i < length * 8; i++) {
      char bit = (buffer[i >> 3] >> (7 - (i & 7))) & 1;
      target = MICRO_CYCLES * (3 - bit * 2);
      while (readTimer() < target);
      N64_HIGH;
      while (readTimer() < MICRO_CYCLES * 4);
      N64_LOW;
      startTimer();
    }

    target = MICRO_CYCLES * (1 + wide_stop);

    while (readTimer() < target);

    N64_HIGH;
    LED_LOW;
    while (!N64_QUERY);
}

/**
  * Waits for an incomming signal on the N64 pin and reads the command,
  * and if necessary, any trailing bytes.
  * 0x00 is an identify request
  * 0x01 is a status request
  * 0x02 is a controller pack read
  * 0x03 is a controller pack write
  *
  * for 0x02 and 0x03, additional data is passed in after the command byte,
  * which is also read by this function.
  *
  * All data is raw dumped to the n64_raw_dump array, 1 bit per byte, except
  * for the command byte, which is placed all packed into n64_command
  */
static void get_n64_command()
{
    int bitcount = 8;
    n64_command = 0;
    char newByte = 0;

    LED_HIGH;

    for (int i = 1; i <= bitcount; i++) {
        while (!N64_QUERY);
        long lowTime = readTimer();
        startTimer();
        while (N64_QUERY);
        long highTime = readTimer();
        startTimer();
        char bit = (lowTime < highTime);
        newByte <<= 1;
        newByte |= bit;

        if (i == 8) {
          n64_command = newByte;
          switch (newByte) {
              case (0x03):
                  // write command
                  // we expect a 2 byte address and 32 bytes of data
                  bitcount += 272; // 34 bytes * 8 bits per byte
                  //Serial.println("command is 0x03, write");
                  break;
              case (0x02):
                  // read command 0x02
                  // we expect a 2 byte address
                  bitcount += 16;
                  //Serial.println("command is 0x02, read");
                  break;
              case (0x00):
              case (0x01):
              default:
                  // get the last (stop) bit
                  break;
          }
        } else if (!(i & 7)) {
          n64_raw_dump[(i >> 3) - 1] = newByte;
        }
    }

    // Wait for the stop bit
    while (!N64_QUERY);
    startTimer();
    while (readTimer() < MICRO_CYCLES * 2);

    LED_LOW;
}
