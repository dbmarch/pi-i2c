#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <errno.h>
#include <string.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "lis2dh12_reg.h"




//#include "gpio.h"
//#include "i2c.h"

/* Private macro -------------------------------------------------------------*/
#define DEV_I2C  "/dev/i2c-1"
#define ACCEL_ADDR 0x19
#define ACCEL_ADDR_2 0x18



/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Private functions ---------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static int platform_init(int devId);

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                     int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}


static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}




int main(int argc, char **argv)
{
  /*
   *  Initialize mems driver interface
   */
  	uint8_t data;
    uint8_t addr = ACCEL_ADDR;
    uint8_t reg = 0xF;

   lis2dh12_ctx_t dev_ctx;
  int fd;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  
  /*
   * Initialize platform specific hardware
   */
  fd = platform_init(addr);
  if (fd < 0)
  {
    printf("unable to open I2C - exit\n");
    return (-1);
  }

  dev_ctx.handle = (void*) fd;

  /*
   *  Check device ID
   */
   
	int rc = ioctl(fd, I2C_SLAVE, addr);
	if (rc < 0)
		printf("Tried to set device address '0x%02x'", addr);


  data = i2c_smbus_read_byte_data(fd, reg);

	printf("%s: device 0x%02x at address 0x%02x: 0x%02x\n",
			DEV_I2C, addr, reg, data);   
   
  //int32_t res;
  //res = i2c_smbus_read_byte_data(fd, 0x0F);
  
  //printf ("i2c_smbus_read_byte returns %02X\n", res);
   
  lis2dh12_device_id_get(&dev_ctx, &whoamI);
  printf("device id: %d\n", whoamI);
  if (whoamI != LIS2DH12_ID)
  {
    printf ("Device not found\n");
    
  } else {
    printf ("Found the LIS2DH12 device\n");
  }
  

  /*
   *  Enable Block Data Update
   */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate to 1Hz
   */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);

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
  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);

  /*
   * Read samples in polling mode (no int)
   */
  while (1)
  {
    lis2dh12_reg_t reg;

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

      sprintf((char *)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);
//    if (reg.byte)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lis2dh12_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC =
          lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);

      sprintf((char *)tx_buffer,
              "Temperature [degC]:%6.2f\r\n",
              temperature_degC);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
    
  }
  return (0);
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  int fd = (int) handle;
  //printf ("Platform_write reg=%02X cnt=%d\n", reg, len);
  
  ioctl(fd, I2C_SLAVE, reg);              
  return write (fd, bufp, len);    

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
  int fd = (int) handle;
  /* Read multiple command */
  //printf ("platform_read reg=%02X, len = %d\n", reg, len);

  ioctl(fd, I2C_SLAVE, reg | 0x80);              
  return read (fd, bufp, len);    

}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  printf("%s\n", tx_buffer);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static int platform_init(int devId)
{
  int fd;
  if ((fd = open(DEV_I2C, O_RDWR)) < 0)
  {
    printf("unable to open I2C: %s\n", strerror(errno));
    return (-1);
    
  }
  if (ioctl(fd, I2C_SLAVE, devId) < 0)
    {
      printf("unable to ioctl I2C_SLAVE\n");
    }
  return fd;
}
