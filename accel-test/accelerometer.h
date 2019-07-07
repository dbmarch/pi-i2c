#include <stdint.h>
#include "lis2dh12_reg.h"

class Accelerometer
{
private:
  int fd;
  lis2dh12_ctx_t dev_ctx;
  char *devPath;
  uint8_t devAddr;

  bool verboseMode;
  bool lowPower;

public:
  Accelerometer();
  int initialize(char *devName, uint8_t devAddr);
  int configure(void);
  void EnableTemperature(float supplyVoltage);
  void EnableDoubleTap();

  bool readTemperature(float &tempC, float &tempF);

  bool readAcceleration(float &x, float &y, float &z);
  bool isDoubleTap(void);
  void setVerbosity(bool isEnabled);

  void dumpAllRegisters(void);
};