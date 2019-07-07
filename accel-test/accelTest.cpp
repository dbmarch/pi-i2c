#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "accelerometer.h"

/* Private macro -------------------------------------------------------------*/
#define DEV_I2C "/dev/i2c-1"
#define ACCEL_ADDR 0x19
#define ACCEL_ADDR_2 0x18

/* Private variables ---------------------------------------------------------*/
static bool temperatureMode = false;
static bool accelMode = false;
static bool doubleTap = false;
static uint8_t devAddr = ACCEL_ADDR;
static bool lowPower = false;
static bool pendingRegisterDump = false;

Accelerometer myAccelerometer;

/* Private functions ---------------------------------------------------------*/

static void usage(void)
{
  printf("d </dev/i2c-0 path to the i2c device\n");
  printf("a <0x18,0x19> address\n");
  printf("v             verbose Mode\n");
  printf("t             temp Mode\n");
  printf("x             accel Mode\n");
  printf("p             doubleTap mode\n");
  printf("l             low power mode\n");
  printf("r             dump all registers\n");
  printf("h             this menu\n");
  exit(0);
}

int main(int argc, char **argv)
{
  /*
   *  Initialize mems driver interface
   */

  int opt;
  lis2dh12_ctx_t dev_ctx;
  char devPath[20];
  int fd;
  int x;

  strcpy(devPath, DEV_I2C);

  opterr = 0;

  while ((opt = getopt(argc, argv, "d:a:vtxphrl")) != -1)
    switch (opt)
    {
    case 'd':
      strcpy(devPath, optarg);
      printf("devPath: %s\n", devPath);
      break;

    case 'a':
      sscanf(optarg, "%d", &x);
      if (x != 0)
        devAddr = x;
      else
      {
        sscanf(optarg, "%x", &x);
        if (x != 0)
          devAddr = x;
      }
      printf("devAddr = %02X\n", devAddr);
      break;

    case 'v':
      printf("Verbose Mode\n");
      myAccelerometer.setVerbosity(true);
      break;

    case 't':
      printf("Temperature Mode\n");
      temperatureMode = true;
      break;

    case 'x':
      printf("Accelerometer Mode\n");
      accelMode = true;
      break;

    case 'p':
      printf("DoubleTap Mode \n");
      doubleTap = true;
      break;

    case 'l':
      printf("Low Power Mode\n");
      lowPower = true;
      break;

    case 'h':
      usage();
      break;

    case 'r':
      pendingRegisterDump = true;
      break;

    default:
      printf("unknown option %c\n", opt);
      break;
    }

  myAccelerometer.initialize(devPath, devAddr);

  if (pendingRegisterDump)
  {
    myAccelerometer.dumpAllRegisters();
  }

  myAccelerometer.configure();

  if (!temperatureMode && !accelMode && !doubleTap)
  {
    printf("No mode selected....\n");
    usage();
  }

  while (1)
  {

    if (accelMode)
    {
      float x, y, z;
      if (myAccelerometer.readAcceleration(x, y, z))
        printf("x %4.1f  y %4.1f  z %4.1f\n", x, y, z);
    }

    if (temperatureMode)
    {
      float tempC, tempF;
      if (myAccelerometer.readTemperature(tempC, tempF))
        printf("TEMP:  %4.2f C  %4.2f F", tempC, tempF);
    }

    if (doubleTap)
    {
      if (myAccelerometer.isDoubleTap())
        printf("<<DOUBLE TAP>\n");
    }
    sleep(1);
  }
  return (0);
}
