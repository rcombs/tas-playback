#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#define INPUT_SIZE 4
#define INPUT_BATCH 8

int main(int argc, const char** argv)
{
  if (argc != 3)
    return 1;
  char dataBuf[INPUT_SIZE * 1024 * 1024];
  int ret;
  int infd = open(argv[1], O_RDONLY);
  int outfd = open(argv[2], O_RDWR | O_NOCTTY);

  ret = read(infd, &dataBuf, sizeof(dataBuf));

  printf("READY : %i\n", ret);

  speed_t baud = B115200;

  struct termios settings;
  tcgetattr(outfd, &settings);
  cfsetispeed(&settings, baud); /* baud rate */
  cfsetospeed(&settings, baud); /* baud rate */
  tcsetattr(outfd, TCSANOW, &settings); /* apply the settings */
  tcflush(outfd, TCOFLUSH);

  char buf[1024];
  char printBuf[1024] = {0};
  char outBuf[1024 * INPUT_SIZE * INPUT_BATCH] = {0};
  int i = 0;
  while ((ret = read(outfd, buf, sizeof(buf))) > 0) {
    int sendCount = 0;
    int readCount = ret;
    for (int i = 0; i < ret; i++) {
      if (buf[i] == 0) {
        sendCount++;
      } else {
        printBuf[i - sendCount] = buf[i];
      }
    }
    if (sendCount) {
      int count = sendCount * INPUT_SIZE * INPUT_BATCH;
      write(outfd, &dataBuf[i], count);
      i += count;
      if (i >= sizeof(dataBuf))
        return 0;
    }
    if (readCount != sendCount) {
      fwrite(printBuf, readCount - sendCount, 1, stdout);
    }
  }
  printf("BAD RET: %i ERRNO: %i\n", ret, errno);
  return 2;
}
