#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu6050.h"
#include "sensorlib/comp_dcm.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_can.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"


#define SLAVE_ADDRESS           0x68
#define AMBIENT_TEMPERATURE     0x3B
#define DEVICE_ID_REGISTER          0x3B
#define MANUFACTURE_ID_REGISTER     0x07

#define GPIO_PB2_I2C0SCL        0x00010801
#define GPIO_PB3_I2C0SDA        0x00010C01

float *pfEulers, *pfQuaternion;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU6050 g_sMPU6050Inst;

//*****************************************************************************
//
// The factors used to convert the acceleration readings from the MPU6050 into
// floating point values in meters per second squared.
//
// Values are obtained by taking the g conversion factors from the data sheet
// and multiplying by 9.81 (1 g = 9.81 m/s^2).
//
//*****************************************************************************
static const float g_fMPU6050AccelFactors[] =
{
    0.00059875,                              // Range = +/- 2 g (16384 lsb/g)
    0.00119751,                              // Range = +/- 4 g (8192 lsb/g)
    0.00239502,                              // Range = +/- 8 g (4096 lsb/g)
    0.00479004                               // Range = +/- 16 g (2048 lsb/g)
};

//*****************************************************************************
//
// The factors used to convert the acceleration readings from the MPU6050 into
// floating point values in radians per second.
//
// Values are obtained by taking the degree per second conversion factors
// from the data sheet and then converting to radians per sec (1 degree =
// 0.0174532925 radians).
//
//*****************************************************************************
static const float g_fMPU6050GyroFactors[] =
{
    1.3323124e-4f,   // Range = +/- 250 dps  (131.0 LSBs/DPS)
    2.6646248e-4f,   // Range = +/- 500 dps  (65.5 LSBs/DPS)
    5.3211258e-4f,   // Range = +/- 1000 dps (32.8 LSBs/DPS)
    0.0010642252f    // Range = +/- 2000 dps (16.4 LSBs/DPS)
};

uint32_t ui32SysClock_2;
int a = 0;
uint32_t pui32DataTx[3];
uint32_t pui32DataRx[3];

float * X;
float sBufAcc[3];
float sBufTemp;
float sBufGyro[3];
float * spBufQuaternion;


uint16_t  UpperByte_X = 0;
uint16_t  LowerByte_X = 0;
uint16_t  UpperByte_Y = 0;
uint16_t  LowerByte_Y = 0;
uint16_t  UpperByte_Z = 0;
uint16_t  LowerByte_Z = 0;

void I2C_Init()
{
	//Разрешить работу I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    //Перезапуск I2C
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    //Включение порта B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Конфигурирование пинов порта B для работы в режие I2C
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    // Конфигурирование Мастер-узла
    I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(), false);
}

void xAxis_R(uint8_t ambient_temp_reg)
{
  I2CMasterDataPut(I2C0_MASTER_BASE, ambient_temp_reg);
  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
}

float * xAxis()
{
  static float sRes[7];
  float fFactor;
  int16_t rtemp_MPU;

  //
  // Get the conversion factor for the current data format.
  //
  fFactor = g_fMPU6050AccelFactors[0];

  I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, SLAVE_ADDRESS, true);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  UpperByte_X = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  LowerByte_X = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  UpperByte_Y = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  LowerByte_Y = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  UpperByte_Z = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  LowerByte_Z = I2CMasterDataGet(I2C0_MASTER_BASE);

  //
  // Convert the Accelerometer values into rad/sec
  //
  sRes[0] = ((float)(short)((UpperByte_X<<8) | LowerByte_X) * fFactor);
  sRes[1] = ((float)(short)((UpperByte_Y<<8) | LowerByte_Y) * fFactor);
  sRes[2] = ((float)(short)((UpperByte_Z<<8) | LowerByte_Z) * fFactor);


  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  UpperByte_X = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  LowerByte_X = I2CMasterDataGet(I2C0_MASTER_BASE);

  rtemp_MPU = (((int16_t) UpperByte_X) << 8) | LowerByte_X;
  sRes[3] = (float)(rtemp_MPU)/340+36.53;

  //
  // Get the conversion factor for the current data format.
  //
  fFactor = g_fMPU6050GyroFactors[0];

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  UpperByte_X = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  LowerByte_X = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  UpperByte_Y = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  LowerByte_Y = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  UpperByte_Z = I2CMasterDataGet(I2C0_MASTER_BASE);

  I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  while(I2CMasterBusy(I2C0_MASTER_BASE));
  LowerByte_Z = I2CMasterDataGet(I2C0_MASTER_BASE);


  sRes[4] = ((float)(short)((UpperByte_X<<8) | LowerByte_X) * fFactor);
  sRes[5] = ((float)(short)((UpperByte_Y<<8) | LowerByte_Y) * fFactor);
  sRes[6] = ((float)(short)((UpperByte_Z<<8) | LowerByte_Z) * fFactor);


  return sRes;

}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port M interrupt event. For this
// application GPIO port M pin 3 is the interrupt line for the MPU9150
//
// For BoosterPack 2 Interface use Port M pin 7.
//
//*****************************************************************************
void
GPIOPortBIntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the status flags to see which pin(s) caused the interrupt.
    //
    ulStatus = GPIOPinIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOPinIntClear(GPIO_PORTB_BASE, ulStatus);

    //
    // Check if this is an interrupt on the MPU6050 interrupt line.
    //
    // For BoosterPack 2 use Pin 7 instead.
    //
    if(ulStatus & GPIO_PIN_7)
    {

        //
        // MPU6050 Data is ready for retrieval and processing.
        //
        //MPU6050DataRead(&g_sMPU6050Inst, MPU6050AppCallback, &g_sMPU6050Inst);
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C0 Interrupt. I2C0 is the I2C connection
// to the MPU6050.
//
//
// For BoosterPack 2 interface change this to the I2C8 interrupt location in
// the vector table in the startup file.
//
//*****************************************************************************
void
MPU6050I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    //I2CMIntHandler(&g_sI2CInst);
}



void init_mpu6050(void)
{

    ui32SysClock_2 = SysCtlClockGet();

    I2C_Init();

    //SysCtlDelay(500);

    pui32DataTx[0] = 0x6B;                                           //MPU6050  PWR_MGMT_1 Register
    pui32DataTx[1] = 0x00;                                           //Set to 0x00 the register PWR_MGMT_1
    I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, SLAVE_ADDRESS, false);
    I2CMasterDataPut(I2C0_MASTER_BASE, pui32DataTx[0] );
    I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_MASTER_BASE)){}
    pui32DataRx[0]=I2CSlaveDataGet(I2C0_MASTER_BASE);

    I2CMasterDataPut(I2C0_MASTER_BASE, pui32DataTx[1] );
    I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_MASTER_BASE)){}
    pui32DataRx[1]=I2CSlaveDataGet(I2C0_MASTER_BASE);

    I2CMasterControl( I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH );//Finish the Burst sending
    while(I2CMasterBusy(I2C0_MASTER_BASE)) {}
    pui32DataRx[2]=I2CSlaveDataGet(I2C0_MASTER_BASE);

    //
    // Initialize the DCM system. 50 hz sample rate.
    // accel weight = .2, gyro weight = .8, mag weight = .2
    //

    CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);

}

float * main_mpu6050(void)
{

	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, SLAVE_ADDRESS, false);
      xAxis_R(0x3B);
      while(I2CMasterBusy(I2C0_MASTER_BASE)){}
      X = xAxis();
      sBufAcc[0] = X[0];
      sBufAcc[1] = X[1];
      sBufAcc[2] = X[2];

      sBufGyro[0] = X[4];
      sBufGyro[1] = X[5];
      sBufGyro[2] = X[6];

      sBufTemp = X[3];


      CompDCMAccelUpdate(&g_sCompDCMInst, sBufAcc[0], sBufAcc[1],
    		  sBufAcc[2]);
      CompDCMGyroUpdate(&g_sCompDCMInst, sBufGyro[0], sBufGyro[1],
    		  sBufGyro[2]);
      CompDCMStart(&g_sCompDCMInst);

      //
      // Get Quaternions.
      //
      spBufQuaternion = CompDCMComputeQuaternion(&g_sCompDCMInst, pfQuaternion);
      X[7] = spBufQuaternion[0];
	  X[8] = spBufQuaternion[1];
  	  X[9] = spBufQuaternion[2];
	  X[10] = spBufQuaternion[3];
      return X;
}
