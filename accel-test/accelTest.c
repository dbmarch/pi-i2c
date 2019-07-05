#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <errno.h>
#include <string.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "lis2dh12_reg.h"


/* Private macro -------------------------------------------------------------*/
#define DEV_I2C  "/dev/i2c-1"
#define ACCEL_ADDR 0x19
#define ACCEL_ADDR_2 0x18

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];
static bool verboseMode = false;
static bool temperatureMode = false;
static bool accelMode = false;
static bool doubleTap = false;
static uint8_t devAddr = ACCEL_ADDR;
static bool lowPower = false;
static bool pendingRegisterDump = false;

/* Private functions ---------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static int platform_init(char* devPath);
static int get_i2c_register(int fd,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char *val);
static int set_i2c_register (int fd,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char val);

static void initAccelerometer( lis2dh12_ctx_t dev_ctx );
static void readTemperature( lis2dh12_ctx_t dev_ctx ) ;
static void readAccelerometer(lis2dh12_ctx_t dev_ctx ) ;
static void readTap (lis2dh12_ctx_t dev_ctx );
static void dumpAllRegisters (lis2dh12_ctx_t dev_ctx );

static void usage(void) {
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
   
   strcpy (devPath, DEV_I2C);


  opterr = 0;

  while ((opt = getopt (argc, argv, "d:a:vtxphr")) != -1)
    switch (opt)
      {
        case 'd':
          strcpy (devPath, optarg);
          printf("devPath: %s\n", devPath);
          break;
        
        case 'a':
          sscanf(optarg, "%d",&x);
          if (x != 0) 
            devAddr = x;
          else {
            sscanf(optarg, "%x",&x);
            if (x != 0) 
               devAddr = x;
          }
          printf("devAddr = %02X\n", devAddr);
          break;
          
        case 'v':
          printf ("Verbose Mode\n");
          verboseMode = true;
          break;
        case 't':
          printf ("Temperature Mode\n");
          temperatureMode = true;
          break;
        
        case 'x': 
          printf ("Accelerometer Mode\n");
          accelMode = true;
          break;
        
        case 'p':
          printf("DoubleTap Mode \n");
          doubleTap = true;
          break;
        
        case 'l':
          lowPower = true;
          break;
          
        case 'h':
          usage();
          break;
        case 'r':
          pendingRegisterDump = true;
          break;
          
        default:
          printf ("unknown option %c\n", opt);
          break;
      }
    
  fd = platform_init(devPath);
  if (fd < 0)
  {
    printf("unable to open I2C - exit\n");
    return (-1);
  }
  
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = (void*) fd;
  
  
  if (pendingRegisterDump) {
    dumpAllRegisters(dev_ctx);
  }
  
  initAccelerometer(dev_ctx);

  
  if (!temperatureMode && !accelMode && !doubleTap) {
    printf("No mode selected....\n");
    usage();
  }
  /*
   * Read samples in polling mode (no int)
   */
  while (1)
  {

    if (accelMode) {
      readAccelerometer(dev_ctx);
    }

    if (temperatureMode) {
      readTemperature(dev_ctx);
    }
    
    if (doubleTap) {
       readTap(dev_ctx);
    }
    sleep(1);
  }
  return (0);
}


static void initAccelerometer( lis2dh12_ctx_t dev_ctx ) {
  
  lis2dh12_device_id_get(&dev_ctx, &whoamI);

  if (verboseMode) {
    printf("device id: %02X\n", whoamI);  
  }
  if (whoamI != LIS2DH12_ID) {
    printf ("Device not found\n");
  } else {
    printf ("Identifed LIS2DH12...\n");
  }
 

  /*
   *  Enable Block Data Update
   */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate to 1Hz
   */
  if (lowPower)
     lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);
  else
     lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_400Hz);
  

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
   
   if(lowPower) 
      lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_LP_8bit);
   else
      lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);
  
  lis2dh12_data_format_set(&dev_ctx, LIS2DH12_LSB_AT_LOW_ADD);

  if (doubleTap) {
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
}

static void dumpAllRegisters (lis2dh12_ctx_t dev_ctx ) {
  int fd = (int) dev_ctx.handle;  
  uint8_t val;
  
  for (int i = 0x1E; i <= 0x3F; i++) {
    get_i2c_register(fd, devAddr, i, &val);    
    printf("[0x%02X] = %02X\n", i, val);
  }
  exit(0);
}


static void readTemperature( lis2dh12_ctx_t dev_ctx ) {
    lis2dh12_reg_t reg;
    axis1bit16_t data_raw_temperature;
    float temperature_degC;
    float temperature_degF;
        
    lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);
    if (reg.byte)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lis2dh12_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC =
          lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);
      
      float scale = 2.5f/3.3f;   // The raspberry pi is operating at 3.3V.  Device calibrated to 2.5V
      
      temperature_degC *=scale;
      
      temperature_degF = (temperature_degC *9 / 5 ) + 32.0;

      sprintf((char *)tx_buffer,
              "Temperature:     %6.2f C     %6.2f F\n",
              temperature_degC , temperature_degF);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      
    }
  
}

static void readAccelerometer ( lis2dh12_ctx_t dev_ctx ) {
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

      sprintf((char *)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

  }
  
  
static void readTap (lis2dh12_ctx_t dev_ctx ) {
  lis2dh12_click_src_t src;
  
  lis2dh12_tap_source_get(&dev_ctx, &src);
  
  printf ("tap: %s %s %s %s %s %s %s\n",
    src.ia ? "IA" : "",
    src.dclick ? "DCLK" : "",
    src.sclick ? "SCLK" : "" ,
    src.sign ? "SIGN" : "",
    src.x ? "X" : "",
    src.y ? "Y" : "",
    src.z ? "Z" : ""
    );
  
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
  int rval = 0;

  if (verboseMode) {
    printf ("WR [%02X]: ", reg);
    for (int i = 0; i < len ; i++) {
      printf("%02X ", bufp[i]);
    }
    printf("\n");
  }

  for (int i = 0; i < len; i++) 
  {
    rval = set_i2c_register(fd, devAddr, reg+i, bufp[i]);
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
  int fd = (int) handle;
  int rval = 0;
  /* Read multiple command */
 
  if (verboseMode)
      printf ("RD [0x%02X]: ", reg);
  
  for (int i = 0; i < len; i++) 
  {
    rval = get_i2c_register(fd, devAddr, reg+i, bufp+i);
    if (verboseMode)
       printf("%02X ", bufp[i]);
  }
  if (verboseMode)
      printf ("\n");
  
    
  return rval;    
  
  
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
  printf("%s", tx_buffer);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static int platform_init(char* devPath)
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
                            unsigned char *val) {
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(fd, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }
    
    *val = inbuf;

    return 0;
}

static int set_i2c_register(int fd,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char val) {
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
    
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = 2;
    messages[0].buf   = outbuf;


    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 1;
    if(ioctl(fd, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }

    return 0;
}

