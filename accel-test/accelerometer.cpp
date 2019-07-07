#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "lis2dh12_reg.h"
#include "accelerometer.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

typedef struct
{
  int fd;
  bool verbose;
  uint8_t addr;
} t_handle;

/* Private functions ---------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

static int platform_init(char *devPath);
static int get_i2c_register(int fd,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char *val);
static int set_i2c_register(int fd,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char val);

Accelerometer::Accelerometer()
    : lowPower(false), verboseMode(false)
{
}

int Accelerometer::initialize(char *devName, uint8_t devAddr)
{

  devPath = new char[sizeof(devName + 1)];
  strcpy(devPath, devName);

  t_handle *pHandle = new t_handle;
  fd = platform_init(devPath);
  if (fd < 0)
  {
    printf("unable to open I2C - exit\n");
    return (-1);
  }
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  pHandle->fd = fd;
  pHandle->verbose = verboseMode;
  pHandle->addr = devAddr;
  dev_ctx.handle = (void *)pHandle;
  return (0);
}

int Accelerometer::configure(void)
{
  uint8_t whoamI;

  lis2dh12_device_id_get(&dev_ctx, &whoamI);

  if (verboseMode)
  {
    printf("device id: %02X\n", whoamI);
  }
  if (whoamI != LIS2DH12_ID)
  {
    printf("Device not found\n");
  }
  else
  {
    printf("Identifed LIS2DH12...\n");
  }

  /*
   *  Enable Block Data Update
   */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate to 1Hz
   */
  if (lowPower)
  {
    //lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);
    lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_400Hz); // 400hz required for double tap.
  }
  else
  {
    lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP);
  }

  /*
   * Set full scale to 2g
   */
  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);

  /*
   * Enable temperature sensor
   */
  lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);

  /*
   * Set device in continuous mode with 12 bit resol.
   */

  if (lowPower)
    lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_LP_8bit);
  else
    lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);

  lis2dh12_data_format_set(&dev_ctx, LIS2DH12_LSB_AT_LOW_ADD);
  return (0);
}

void Accelerometer::EnableDoubleTap(void)
{
  lis2dh12_click_cfg_t tapcfg;
  tapcfg.xs = 1;
  tapcfg.xd = 1;
  tapcfg.ys = 1;
  tapcfg.yd = 1;
  tapcfg.zs = 1;
  tapcfg.zd = 1;

  lis2dh12_tap_conf_set(&dev_ctx, &tapcfg);

  lis2dh12_tap_notification_mode_set(&dev_ctx, LIS2DH12_TAP_LATCHED);
  lis2dh12_act_timeout_set(&dev_ctx, 0x0f);

  lis2dh12_tap_threshold_set(&dev_ctx, 0x05);
  lis2dh12_shock_dur_set(&dev_ctx, 5);
  lis2dh12_quiet_dur_set(&dev_ctx, 0x24);
  lis2dh12_double_tap_timeout_set(&dev_ctx, 0x7f);
}

void Accelerometer::dumpAllRegisters(void)
{
  int fd = (int)dev_ctx.handle;
  uint8_t val;

  for (int i = 0x1E; i <= 0x3F; i++)
  {
    get_i2c_register(fd, devAddr, i, &val);
    printf("[0x%02X] = %02X\n", i, val);
  }
  exit(0);
}

bool Accelerometer::readTemperature(float &tempC, float &tempF)
{
  lis2dh12_reg_t reg;
  axis1bit16_t data_raw_temperature;
  float temperature_degC = 0;
  float temperature_degF = 0;

  lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);
  if (reg.byte)
  {

    /* Read temperature data */
    memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    lis2dh12_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
    temperature_degC =
        lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);

    float scale = 2.5f / 3.3f; // The raspberry pi is operating at 3.3V.  Device calibrated to 2.5V

    temperature_degC *= scale;

    temperature_degF = (temperature_degC * 9 / 5) + 32.0;

    if (verboseMode)
    {
      printf("Temperature:     %6.2f C     %6.2f F\n",
             temperature_degC, temperature_degF);
    }

    tempC = temperature_degC;
    tempF = temperature_degF;
    return (true);
  }
  return (false);
}

bool Accelerometer::readAcceleration(float &x, float &y, float &z)
{
  lis2dh12_reg_t reg;
  axis3bit16_t data_raw_acceleration;
  float acceleration_mg[3];

  /*
     * Read output only if new value available
     */
  lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
  if (reg.byte)
  {
    /* Read accelerometer data */
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
    acceleration_mg[0] =
        lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[0]);
    acceleration_mg[1] =
        lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[1]);
    acceleration_mg[2] =
        lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[2]);

    if (verboseMode)
    {
      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\n",
             acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    }
    x = acceleration_mg[0];
    y = acceleration_mg[1];
    z = acceleration_mg[2];
    return (true);
  }
  return (false);
}

void Accelerometer::setVerbosity(bool isEnabled)
{
  verboseMode = isEnabled;
  printf("setVerbosity %d\n", verboseMode);
}

bool Accelerometer::isDoubleTap(void)
{
  lis2dh12_click_src_t src;

  lis2dh12_tap_source_get(&dev_ctx, &src);

  printf("tap: %s %s %s %s %s %s %s\n",
         src.ia ? "IA" : "",
         src.dclick ? "DCLK" : "",
         src.sclick ? "SCLK" : "",
         src.sign ? "SIGN" : "",
         src.x ? "X" : "",
         src.y ? "Y" : "",
         src.z ? "Z" : "");

  return (src.dclick != 0);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t
platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  t_handle *i2cHandle = (t_handle *)handle;
  int fd = i2cHandle->fd;
  bool verboseMode = i2cHandle->verbose;
  uint8_t devAddr = i2cHandle->addr;

  int rval = 0;

  if (verboseMode)
  {
    printf("WR [%02X]: ", reg);
    for (int i = 0; i < len; i++)
    {
      printf("%02X ", bufp[i]);
    }
    printf("\n");
  }

  for (int i = 0; i < len; i++)
  {
    rval = set_i2c_register(fd, devAddr, reg + i, bufp[i]);
  }

  return rval;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  t_handle *i2cHandle = (t_handle *)handle;
  int fd = i2cHandle->fd;
  bool verboseMode = i2cHandle->verbose;
  uint8_t devAddr = i2cHandle->addr;

  int rval = 0;
  /* Read multiple command */

  if (verboseMode)
    printf("RD [0x%02X]: ", reg);

  for (int i = 0; i < len; i++)
  {
    rval = get_i2c_register(fd, devAddr, reg + i, bufp + i);
    if (verboseMode)
      printf("%02X ", bufp[i]);
  }
  if (verboseMode)
    printf("\n");

  return rval;
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static int platform_init(char *devPath)
{
  int fd;
  if ((fd = open(devPath, O_RDWR)) < 0)
  {
    printf("unable to open I2C: %s\n", strerror(errno));
    return (-1);
  }
  return fd;
}

static int get_i2c_register(int fd,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char *val)
{
  unsigned char inbuf, outbuf;
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];

  /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
  outbuf = reg;
  messages[0].addr = addr;
  messages[0].flags = 0;
  messages[0].len = sizeof(outbuf);
  messages[0].buf = &outbuf;

  /* The data will get returned in this structure */
  messages[1].addr = addr;
  messages[1].flags = I2C_M_RD /* | I2C_M_NOSTART*/;
  messages[1].len = sizeof(inbuf);
  messages[1].buf = &inbuf;

  /* Send the request to the kernel and get the result back */
  packets.msgs = messages;
  packets.nmsgs = 2;
  if (ioctl(fd, I2C_RDWR, &packets) < 0)
  {
    perror("Unable to send data");
    return 1;
  }

  *val = inbuf;

  return 0;
}

static int set_i2c_register(int fd,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char val)
{
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];
  unsigned char outbuf[2];
  outbuf[0] = reg;
  outbuf[1] = val;

  /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */

  messages[0].addr = addr;
  messages[0].flags = 0;
  messages[0].len = 2;
  messages[0].buf = outbuf;

  /* Send the request to the kernel and get the result back */
  packets.msgs = messages;
  packets.nmsgs = 1;
  if (ioctl(fd, I2C_RDWR, &packets) < 0)
  {
    perror("Unable to send data");
    return 1;
  }

  return 0;
}
