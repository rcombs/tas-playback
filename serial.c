/*
 * Copyright (c) 2017 Rodger Combs
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#include "config.h"

char dataBuf[INPUT_SIZE * 1024 * 1024 * 8] = {0};

int writec(int fd, char c)
{
  write(fd, &c, 1);
}

void write_7bit(int fd, const char *in, int len)
{
  char out[1024] = {0};
  unsigned char check = 0;
  for (int i = 0; i < len; i++) {
    out[(i * 8) / 7]    |=  in[i] >> 1 + (i % 7);
    out[(i * 8) / 7 + 1] = (in[i] << 7 - (i % 7)) & 0x7F;
    check ^= in[i];
  }
  out[(len * 8) / 7]    |=  check >> 1 + (len % 7);
  out[(len * 8) / 7 + 1] = (check << 7 - (len % 7)) & 0x7F;
  write(fd, out, ((len + 1) * 8 + 6) / 7);
}

int main(int argc, const char** argv)
{
  if (argc != 3)
    return 1;
  int ret, size;
  int infd = open(argv[1], O_RDONLY);
  int outfd = open(argv[2], O_RDWR | O_NOCTTY);

  if (infd < 0 || outfd < 0)
    return 1;

  setvbuf(stdout, NULL, _IOLBF, 1024);

  ret = read(infd, &dataBuf, sizeof(dataBuf));

  if (ret < 0)
    return 1;

  size = ret;

  printf("Total size: %i\n", size);

  speed_t baud = B115200;

  struct termios settings;
  tcgetattr(outfd, &settings);
  settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  settings.c_oflag &= ~OPOST;
  settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  settings.c_cflag &= ~(CSIZE | PARENB);
  settings.c_cflag |= CS8;
  cfsetispeed(&settings, baud); /* baud rate */
  cfsetospeed(&settings, baud); /* baud rate */
  settings.c_cc[VMIN] = 1;
  settings.c_cc[VTIME] = 0;
  tcsetattr(outfd, TCSANOW, &settings); /* apply the settings */
  tcflush(outfd, TCOFLUSH);

  char buf[1024];
  char printBuf[1024] = {0};
  char outBuf[1024 * INPUT_SIZE * INPUT_BATCH] = {0};
  int i = HEADER_SIZE;
  while ((ret = read(outfd, buf, sizeof(buf))) > 0) {
    int readCount = ret;
    int printLen = 0;
    for (int j = 0; j < ret; j++) {
      if (buf[j] == REQ_BYTE) {
        writec(outfd, START_BYTE);
        for (int k = 0; k < INPUT_BATCH; k++)
          write_7bit(outfd, &dataBuf[i + k * INPUT_SIZE], INPUT_SIZE);
      } else if (buf[j] == ACK_BYTE) {
        i += INPUT_SIZE;
        if (i >= size + (INPUT_SIZE * (INPUT_BATCH - 1))) {
          writec(outfd, FINISH_BYTE);
          return 0;
        }
      } else {
        printBuf[printLen++] = buf[j];
      }
    }
    if (printLen)
      fwrite(printBuf, printLen, 1, stdout);
  }
  printf("BAD RET: %i ERRNO: %i\n", ret, errno);
  return 2;
}
