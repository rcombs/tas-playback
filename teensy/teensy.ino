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
#include <MinimumSerial.h>
#include <SdFat.h>
#include <BlockDriver.h>
#include <SysCall.h>
#include <DMAChannel.h>

#define STATUS_PIN 13

#define SERIAL_BAUD_RATE 115200

#define N64_PIN 2

#define N64_HIGH (CORE_PIN2_DDRREG &= ~CORE_PIN2_BITMASK) //digitalWriteFast(N64_PIN, HIGH)
#define N64_LOW (CORE_PIN2_DDRREG |= CORE_PIN2_BITMASK) //digitalWriteFast(N64_PIN, LOW)
#define N64_QUERY ((CORE_PIN2_PINREG & CORE_PIN2_BITMASK) ? 1 : 0) //digitalReadFast(N64_PIN)

#define LED_HIGH (CORE_PIN13_PORTSET = CORE_PIN13_BITMASK) //digitalWriteFast(STATUS_PIN, HIGH)
#define LED_LOW (CORE_PIN13_PORTCLEAR = CORE_PIN13_BITMASK) //digitalWriteFast(STATUS_PIN, LOW)

#define INPUT_BUFFER_SIZE 2048 // Multiples of 512 are ideal since we can read 256*4/2 = 512 bytes at once.
// 512 bytes is an optimization for reading the sd card and skips using another buffer.

#define INPUT_BUFFER_UPDATE_TIMEOUT 10 // 10 ms

template<class T> void lockedPrintln(const T& input)
{
  noInterrupts();
  Serial.println(input);
  interrupts();
}

template<class A, class B> void lockedPrintln(const A& a, const B& b)
{
  noInterrupts();
  Serial.print(a);
  Serial.println(b);
  interrupts();
}

static char n64_raw_dump[36]; // maximum recv is 1+2+32 bytes + 1 bit
// n64_raw_dump does not include the command byte. That gets pushed into
// n64_command:
static int n64_command;
// bytes to send to the 64
// maximum we'll need to send is 33, 32 for a read request and 1 CRC byte
static unsigned char output_buffer[33];
static void get_n64_command();
static void n64_send(unsigned char *buffer, char length, bool wide_stop);

// Simple switch buffer. (If buffer A fails to load while buffer B is in use,
// we still okay, and will try again next loop)
static char inputBuffer[INPUT_BUFFER_SIZE];
static size_t bufferEndPos;
static volatile size_t bufferPos;
static volatile bool bufferHasData;
static void updateInputBuffer();

static bool hold = false;
static bool finished = false;

static size_t getNextRegion(size_t& size) {
  size_t currentPos = bufferPos;
  if ((currentPos == bufferEndPos || (currentPos == 0 && bufferEndPos == INPUT_BUFFER_SIZE)) && bufferHasData) {
    size = 0;
    return 0;
  }
  size_t ret = bufferEndPos;
  if (ret == INPUT_BUFFER_SIZE)
    ret = 0;
  size = INPUT_BUFFER_SIZE - ret;
  if (ret < currentPos)
    size = currentPos - ret;
  return ret;
}

void initBuffers()
{
  bufferEndPos = 0;
  bufferPos = 0;
  bufferHasData = false;
}

static void advanceBuffer(long bytes)
{
  size_t pos = bufferPos + bytes;
  if (pos == bufferEndPos) {
    if (hold)
      return;
    bufferHasData = false;
    bufferPos = pos;
    return;
  }
  if (pos >= INPUT_BUFFER_SIZE)
      pos = 0;
  bufferPos = pos;
}

static size_t appendInputs(const String& data)
{
  size_t totalWritten = 0;
  size_t writeLen = 0;
  do {
    size_t dataLeft = data.length() - totalWritten;
    size_t writePos = getNextRegion(writeLen);
    if (writeLen > dataLeft)
      writeLen = dataLeft;
    if (writeLen) {
      memcpy(inputBuffer + writePos, data.c_str() + totalWritten, writeLen);
      totalWritten += writeLen;
      noInterrupts();
      finished = false;
      bufferEndPos = writePos + writeLen;
      bufferHasData = true;
      interrupts();
    }
  } while (writeLen > 0);
  return totalWritten;
}

static FatFile m64File;
static bool m64OpenSuccess = false;
static bool openM64(const String& path);

//static unsigned int progressPos = 0;
static unsigned long numFrames = 0, curFrame = 0;
static int edgesRead = 0;
static int incompleteCommand = 0;

SdFatSdioEX sd;

#define MICRO_CYCLES (F_CPU / 1000000)

static void emitList(const String& path)
{
  FatFile dir;
  if (!dir.open(&sd, path.c_str(), O_READ)) {
    Serial.println(F("E:Failed to open requested directory"));
    return;
  }
  dir.rewind();
  while (m64File.openNext(&dir)) {
    char name[256];
    m64File.getName(name, sizeof(name));
    lockedPrintln("A:", name);
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
    Serial.println(F("E:SD initialization failed!"));
    return;
  }
  Serial.println(F("L:SD initialization done."));

  // Setup buffer
  initBuffers();

  Serial.println(F("L:Initialization done."));
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

static bool setPinMode(const String& cmd)
{
  if (cmd.length() < 3)
    return false;
  if (cmd[1] != ':')
    return false;

  int pinNumber = cmd.substring(2).toInt();

  if (cmd[0] == 'I')
    pinMode(pinNumber, INPUT);
  else if (cmd[0] == 'O')
    pinMode(pinNumber, OUTPUT);
  else if (cmd[0] == 'U')
    pinMode(pinNumber, INPUT_PULLUP);
  else if (cmd[0] == 'D')
    pinMode(pinNumber, INPUT_PULLDOWN);
  else
    return false;

  return true;
}

static bool writePin(const String& cmd)
{
  if (cmd.length() < 3)
    return false;
  if (cmd[1] != ':')
    return false;

  int pinNumber = cmd.substring(2).toInt();

  if (cmd[0] == '0')
    digitalWrite(pinNumber, LOW);
  else if (cmd[0] == '1')
    digitalWrite(pinNumber, HIGH);
  else
    return false;

  return true;
}

String hextobin(const String& hex)
{
  String ret;
  ret.reserve((hex.length() + 1) / 2);

  // mapping of ASCII characters to hex values
  static const uint8_t map[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // 01234567
    0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 89:;<=>?
    0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, // @ABCDEFG
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // HIJKLMNO
  };

  for (size_t pos = 0; pos < hex.length(); pos += 2) {
    size_t idx0 = ((uint8_t)hex[pos]     & 0x1F) ^ 0x10,
           idx1 = ((uint8_t)hex[pos + 1] & 0x1F) ^ 0x10;
    ret += (char)((map[idx0] << 4) | map[idx1]);
  }

  return ret;
}

static FatFile writeFile;

static void appendData(const String& data) {
  if (writeFile.write(data.c_str(), data.length()) == (int)data.length())
    Serial.println("AP:OK");
  else
    Serial.println("AP:NAK");
}

bool skippingInput = false;
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
    lockedPrintln("MK:", sd.mkdir(cmd.substring(3).c_str()));
  } else if (cmd.startsWith("CR:")) {
    if (writeFile.isOpen()) {
      writeFile.close();
    }
    if (writeFile.open(&sd, cmd.substring(3).c_str(), O_WRITE | O_CREAT | O_TRUNC)) {
      lockedPrintln("CR:OK");
    } else {
      lockedPrintln("CR:NAK");
    }
  } else if (cmd.startsWith("AP:")) {
    appendData(hextobin(cmd.substring(3)));
  } else if (cmd.startsWith("IN:")) {
    size_t len = appendInputs(hextobin(cmd.substring(3)));
    lockedPrintln("IN:", len);
  } else if (cmd.startsWith("HL:")) {
    hold = cmd[3] == '1';
  } else if (cmd.startsWith("CL:")) {
    writeFile.close();
    lockedPrintln("CL:OK");
  } else if (cmd.startsWith("PM:")) {
    setPinMode(cmd.substring(3));
  } else if (cmd.startsWith("DW:")) {
    writePin(cmd.substring(3));
  } else {
    lockedPrintln("E:Unknown CMD:", cmd.c_str());
  }
}

#define MAX_CMD_LEN 4096
#define MAX_LOOP_LEN 10240

static void inputLoop()
{
  int newChar;
  int charsRead = 0;
  while ((newChar = Serial.read()) != -1 && (charsRead++ <= MAX_LOOP_LEN)) {
    if (newChar == '\n') {
      if (skippingInput)
        Serial.println("E:Skipped too-long line");
      else
        handleCommand(inputString);
      inputString = "";
      skippingInput = false;
    } else if (inputString.length() > MAX_CMD_LEN) {
      skippingInput = true;
    } else {
      inputString += (char)newChar;
    }
  }
}

static void waitForIdle(int us)
{
  LED_HIGH;
  N64_HIGH;
  startTimer();
  int lastReset = 0;
  while (readTimer() - lastReset < us * MICRO_CYCLES && readTimer() < us * MICRO_CYCLES * 4) {
    if (!N64_QUERY)
      lastReset = readTimer();
  }
  LED_LOW;
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
    noInterrupts();

    unsigned char data, addr;
    int ticksSinceLast = readTimer();
    startTimer();
    volatile uint32_t *config = portConfigRegister(N64_PIN);
    uint32_t oldConfig = *config;
    bool haveFrame;
//    unsigned int curPos;

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
            // mutilate the output_buffer array with our status
            // we return 0x050001 to indicate we have a rumble pack
            // or 0x050002 to indicate the expansion slot is empty
            //
            // 0xFF I've seen sent from Mario 64 and Shadows of the Empire.
            // I don't know why it's different, but the controllers seem to
            // send a set of status bytes afterwards the same as 0x00, and
            // it won't work without it.
            output_buffer[0] = 0x05;
            output_buffer[1] = 0x00;
            output_buffer[2] = 0x01;

            n64_send(output_buffer, 3, 1);
            Serial.println("P:");
            break;
        case 0x01:
            haveFrame = (!finished && bufferHasData);
            // If the TAS is finished, there's nothing left to do.
            if (haveFrame)
              memcpy(output_buffer, inputBuffer + bufferPos, 4);
            else
              memset(output_buffer, 0, 4);
        
            // blast out the pre-assembled array in output_buffer
            n64_send(output_buffer, 4, 1);

            if (!haveFrame)
              break;

            curFrame++;

            logFrame();

            // update input buffer and make sure it doesn't take too long

           /*Serial.print(F("Pos: "));
            Serial.print(bufferPos);
            Serial.print(F(" Data: "));
            Serial.println(inputBuffer[bufferPos]);*/
            
            if (curFrame == numFrames) {
              Serial.println("C:");
              finished = true;
            }

            advanceBuffer(4);

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
            memset(output_buffer, 0x80, 32);
            output_buffer[32] = 0xB8; // CRC

            n64_send(output_buffer, 33, 1);
            Serial.println(F("L:Got a read, what?"));

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
            output_buffer[0] = crc_repeating_table[data] ^ 0xFF;

            // send it
            n64_send(output_buffer, 1, 1);
            Serial.println(F("L:Got a write, what?"));

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

        case -1:
            if (finished)
              break;
            Serial.print(F("W:Read timeout; "));
            Serial.print(edgesRead);
            Serial.println(F(" edges read"));
            if (edgesRead >= 2) {
              Serial.print(F("W: partial byte: "));
              Serial.println(incompleteCommand, HEX);
            }
            Serial.print(F("W:"));
            Serial.print(ticksSinceLast);
            Serial.println(F(" cycles since last"));
            if (curFrame > 100)
              Serial.println(F("D:timeout"));
            waitForIdle(1000);
            break;

        default:
            if (finished)
              break;
            Serial.print(F("W:Unknown command: 0x"));
            Serial.println(n64_command, HEX);
            if (curFrame > 100)
              Serial.println(F("D:inval"));
            waitForIdle(1000);
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
    Serial.print(F("L:Opening file '"));
    Serial.print(path);
    Serial.println(F("'..."));

    Serial.flush();

    if (m64File.isOpen())
      m64File.close();
  
    // Error check
    if (!m64File.open(&sd, path.c_str(), O_READ)) {
        Serial.println(F("E:Error in opening file"));
        return false;
    }
  
    // Open header
    if (m64File.read(signature, 4) != 4 || m64File.read(&version, 4) != 4) {
        m64File.close();
        Serial.println(F("E:Failed to read signature"));
        return false;
    }
  
    // Validate file signature
    if (memcmp(signature, "M64\x1A", 4) != 0) {
        Serial.println(F("E:m64 signature invalid"));
        m64File.close();
        return false;
    }
      
    // Print version
    Serial.print(F("L:M64 Version: "));
    Serial.println(version);

    Serial.flush();

    m64File.seekSet(0x018);
  
    // Open header
    uint32_t newNumFrames;
    if (m64File.read(&newNumFrames, 4) != 4) {
        m64File.close();
        Serial.println(F("E:Failed to read frame count"));
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
            Serial.println(F("E:unknown M64 version"));
            m64File.close();
            return false;
    }
  
    // Final check
    if (!m64File.available()) {
        Serial.println(F("E:No input data found in file"));
        m64File.close();
        return false;
    }

    newNumFrames = min(newNumFrames, (m64File.fileSize() - m64File.curPosition()) / 4);

    Serial.write("N:");
    Serial.println(newNumFrames);
    Serial.flush();

    // Wait for the line to go idle, then begin listening
    /*
    for (int idle_wait=32; idle_wait>0; --idle_wait) {
        if (!N64_QUERY) {
            idle_wait = 32;
        }
    }*/

    noInterrupts();
    m64OpenSuccess = true;
    finished = false;
    initBuffers();
    curFrame = 0;
    numFrames = newNumFrames;
    interrupts();
 
    return true;
}

static void updateInputBuffer() {
    if (finished || !m64File.isOpen())
      return;
  
    // Check for file end
    if (!m64File.available()) {
        m64File.close();
        return;
    }

    size_t availableSize;
    size_t writePos = getNextRegion(availableSize);

    // Optimized chunk reads
    if ((availableSize < 512 && (writePos % 512) == 0) || !availableSize)
        return;

    int readBytes = m64File.read(inputBuffer + writePos, availableSize);
    if (readBytes <= 0) {
      lockedPrintln(F("W:Failed to read next inputs from file. (This is recoverable)"));
      return;
    }

    noInterrupts();
    bufferEndPos = writePos + readBytes;
    bufferHasData = true;
    interrupts();
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
  startTimer();
  while (!N64_QUERY && readTimer() < MICRO_CYCLES * 4);
  LED_LOW;
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

#define FAIL_TIMEOUT {\
  if (readTimer() >= MICRO_CYCLES * 10) {\
    n64_command = -1; \
    goto fail; \
  } \
}\

    edgesRead = 0;

    for (int i = 1; i <= bitcount; i++) {
        while (!N64_QUERY)
          FAIL_TIMEOUT;
        long lowTime = readTimer();
        startTimer();
        edgesRead++;
        while (N64_QUERY)
          FAIL_TIMEOUT;
        long highTime = readTimer();
        startTimer();
        edgesRead++;
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
    while (!N64_QUERY)
      FAIL_TIMEOUT;
    startTimer();
    while (readTimer() < MICRO_CYCLES * 2);

fail:
    incompleteCommand = newByte;

    LED_LOW;
}
