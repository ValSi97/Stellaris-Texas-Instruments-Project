#include "EK-LM3S8962-HW-Main.h"
#include "utils/uartstdio.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_can.h"
#include "inc/hw_types.h"
#include "driverlib/can.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "drivers/rit128x96x4.h"
#include "CO_driver.h"
#include "CANopen.h"
#include "debug_info.h"
#include "mpu6050_sensors.h"

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with smaller values indicating higher priority interrupts.
//
//*****************************************************************************
#define SET_INT_PRIORITY(i)     (i << 5)
#define CAN_INT_PRIORITY        SET_INT_PRIORITY(0)    // highest priority
#define TIMER0B_INT_PRIORITY    SET_INT_PRIORITY(1)    // lower priority

//*****************************************************************************
//
//! Counter to count the number of timer interrupts that have been called.
//
//*****************************************************************************
volatile unsigned long g_ulCounter = 0;

//*****************************************************************************
//
//! This function sets up Timer0B to be used as a 1ms CANopen processing timer.
//!
//! \attention
//!       The SysTick interrupt \b *cannot* be used for the 1ms timer because
//!       the CANopenNode code disables the timer interrupts around critical
//!       regions. If a SysTick interrupt is disabled when its next periodic
//!       interrupt is due, the interrupt will be lost, not pended!
//!       However, a standard timer interrupt will only be delayed while its
//!       interrupt is disabled, and will occur as soon as its interrupt
//!       is reenabled.
//!
//! \return None.
//
//*****************************************************************************
static void InitTimer0B(void)
{
    //
    // The Timer0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Configure Timer0B as a 16-bit periodic timer.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_B_PERIODIC);

    //
    // Set the Timer0B load value to 1ms.
    //
    TimerLoadSet(TIMER0_BASE, TIMER_B, SysCtlClockGet() / 1000);

    //
    // Clear any pending interrupt flag.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //
    // Configure the Timer0B interrupt for timer timeout.
    //
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //
    // Enable the Timer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER0B);

    //
    // Enable Timer0B.
    //
    TimerEnable(TIMER0_BASE, TIMER_B);
}

//*****************************************************************************
//
//! The interrupt handler for the Timer0B interrupt.
//
//*****************************************************************************
void Timer0BIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //
    // Update the Timer0B interrupt counter.
    //
    g_ulCounter++;

    CO_Timer1msIsr();

if ((g_ulCounter & 0x0000001f) == 0)
{
    TOGGLE_STATUS_LED();
}
}

//*****************************************************************************
//
//! Configure the CAN and enter a loop to implement CANopen processing.
//
//*****************************************************************************
int main(void)
{

    //
    // Set the clocking to run from the PLL.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
            SYSCTL_XTAL_8MHZ);

    //
    // Disable interrupts to the processor.
    //
    IntMasterDisable();


#ifdef UART_DEBUG
    //
    // Set up the serial console to use for displaying messages.
    //
    InitConsole();
    UARTprintf("\n\nCANopenNode Tutorial Sensor\n");
    UARTprintf("==========================\n");
#endif  // UART_DEBUG

    //
    // Initialize the OLED display.
    //
    RIT128x96x4Init(1000000);


    RIT128x96x4StringDraw("Accel X", 0, 0, 15);
    RIT128x96x4StringDraw("Accel Y", 0, 8, 15);
    RIT128x96x4StringDraw("Accel Z", 0, 16, 15);
    RIT128x96x4StringDraw("Gyro X", 0, 24, 15);
    RIT128x96x4StringDraw("Gyro Y", 0, 32, 15);
    RIT128x96x4StringDraw("Gyro Z", 0, 40, 15);
    RIT128x96x4StringDraw("Quat. W", 0, 48, 15);
    RIT128x96x4StringDraw("Quat. X", 0, 56, 15);
    RIT128x96x4StringDraw("Quat. Y", 0, 64, 15);
    RIT128x96x4StringDraw("Quat. Z", 0, 72, 15);
    RIT128x96x4StringDraw("Temp", 0, 80, 15);
    RIT128x96x4StringDraw("State: ", 0, 88, 15);

    SELECT_SW_INIT();
    LEFT_SW_INIT();
    RIGHT_SW_INIT();

    UP_SW_INIT();
    DOWN_SW_INIT();

    STATUS_LED_INIT();

    // setup Timer0B interrupt
    InitTimer0B();

    CO_DriverInit();        // CANopenNode driver init
    User_Init();            // User init
    CO_ResetComm();         // Reset CAN communication

    //
    // Set the priorities of the interrupts used by the application.
    //
    IntPrioritySet(INT_CAN0, CAN_INT_PRIORITY);
    IntPrioritySet(INT_TIMER0B, TIMER0B_INT_PRIORITY);

    //
    // Enable interrupts to the processor.
    //
    IntMasterEnable();

    init_mpu6050();

    while (1)
    {
        CO_ProcessMain();
        User_ProcessMain();
    }
}

Модуль БЧЭ – BlockOfSensors.c

// CAN board is specified in Makefile
#include <stdlib.h>
#include <math.h>
#include "EK-LM3S8962-HW-Main.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "inc/hw_types.h"
#include "drivers/rit128x96x4.h"

#include "mpu6050_sensors.h"

#include "CANopen.h"
#include "debug_info.h"


/***** Device profile for Generic I/O *****************************************/
           #ifdef CO_IO_DIGITAL_INPUTS
/*0x6000*/ extern       tData4bytes    ODE_Read_Digital_Input;
           #endif

           #ifdef CO_IO_DIGITAL_OUTPUTS
/*0x6200*/ extern       tData4bytes    ODE_Write_Digital_Output;
           #endif

           #ifdef CO_IO_ANALOG_INPUTS
/*0x6401*/ extern       INTEGER16      ODE_Read_Analog_Input[];
           #endif

           #ifdef CO_IO_ANALOG_OUTPUTS
/*0x6411*/ extern       INTEGER16      ODE_Write_Analog_Output[];
           #endif

//#ifdef CAN_DEVICE_LM3S8962
static int TheSelectSwitch = 0;
static int TheLeftSwitch = 0;
static int TheRightSwitch = 0;
//#endif
static int TheUpSwitch = 0;
static int TheDownSwitch = 0;


union
{
	float bufTemp;
	unsigned long ul;
	unsigned char uc[32];
} tempUn;

union
{
	float bufAccel[3];
	unsigned long ul;
	unsigned char uc[32];
} accelUn;

union
{
	float bufGyro[3];
	unsigned long ul;
	unsigned char uc[32];
} gyroUn;

union
{
	float bufQuaternion[4];
	unsigned long ul;
	unsigned char uc[32];
} quatUn;

enum
{
    TEMPERATURE,
	RESET,
    QUATERNION,
	ACCEL,
    GYRO,
} eState;

/*****************************************************************************//**
   SwitchOffNode - SWITCH OFF ALL NODE OUTPUTS

   Function is called after Node startup, Communication reset or after
   NMT_OPERATIONAL (this or monitored nodes) was lost.
*******************************************************************************/
void SwitchOffNode(void)
{
   #ifdef CO_IO_DIGITAL_OUTPUTS
      //CHANGE THIS LINE -> WRITE 0 TO ALL PORTS !!!
      ODE_Write_Digital_Output.DWORD[0] = 0;
   #endif

   #ifdef CO_IO_ANALOG_OUTPUTS
      //CHANGE THIS LINE -> WRITE 0 TO ALL PORTS !!!
      ODE_Write_Analog_Output[0] = 0;
      ODE_Write_Analog_Output[1] = 0;
   #endif
}

/*****************************************************************************//**
   User_Init - USER INITIALIZATION OF NODE
   
   Function is called after start of program.
*******************************************************************************/
void User_Init(void)
{
    ODE_EEPROM.PowerOnCounter++;

#ifdef CO_IO_DIGITAL_INPUTS
    //CHANGE THIS LINE -> set ports as digital inputs
    ODE_Read_Digital_Input.DWORD[0] = 0;
#endif

#ifdef CO_IO_ANALOG_INPUTS
    //CHANGE THIS LINE -> set ports as analog inputs
    ODE_Read_Analog_Input[0] = 0;
    ODE_Read_Analog_Input[1] = 0;
    ODE_Read_Analog_Input[2] = 0;
    ODE_Read_Analog_Input[3] = 0;
    ODE_Read_Analog_Input[4] = 0;
    ODE_Read_Analog_Input[5] = 0;
    ODE_Read_Analog_Input[6] = 0;
    ODE_Read_Analog_Input[7] = 0;
#endif

#ifdef CO_IO_DIGITAL_OUTPUTS
    //CHANGE THIS LINE -> set ports as digital outputs
#endif

#ifdef CO_IO_ANALOG_OUTPUTS
    //CHANGE THIS LINE -> set ports as analog outputs
#endif

    SwitchOffNode();
}

/*****************************************************************************//**
   User_Remove - USER EXECUTION ON EXIT OF PROGRAM
   
   Function is called before end of program. Not generally used.
*******************************************************************************/
void User_Remove(void)
{

}

/*****************************************************************************//**
   User_ResetComm - USER RESET COMMUNICATION
   
   Function is called after start of program and after CANopen NMT command: 
   \b Reset
   \b Communication.
*******************************************************************************/
void User_ResetComm(void)
{
    SwitchOffNode();
}

char cbuf[100];
char* ftoa(float f)
{
	int pos=0,ix,dp,num;

	if ((f == +INFINITY) || (f == -INFINITY))
			f = 0;
	if (f<0)
		{
			cbuf[pos++]='-';
			f = -f;
		}
	dp=0;
	while (f>=10.0)
	{
		f=f/10.0;
		dp++;
	}
	for (ix=1;ix<8;ix++)
	{
		num = (int)f;
		f=f-num;
		if (num>9)
			cbuf[pos++]='#';
		else
			cbuf[pos++]='0'+num;
		if (dp==0) cbuf[pos++]='.';
		f=f*10.0;
		dp--;
	}
	return cbuf;
}

/*****************************************************************************//**
   User_ProcessMain - USER PROCESS MAINLINE
   
   This function is cyclically called from main(). It is a non blocking function.
   It is asynchronous. This is for longer and time consuming code.
*******************************************************************************/
void User_ProcessMain(void)
{
    static int sav_up_sw = 0;
    static int sav_down_sw = 0;
//#ifdef CAN_DEVICE_LM3S8962
    static int sav_select_sw = 0;
    static int sav_left_sw = 0;
    static int sav_right_sw = 0;

    float * buf;

    static unsigned char state = 99;
//#endif  // CAN_DEVICE_LM3S8962
    
//#ifdef CAN_DEVICE_LM3S8962
    if (sav_select_sw != TheSelectSwitch)
    {
        sav_select_sw = TheSelectSwitch;
        if (sav_select_sw)
        {
/*#ifdef UART_DEBUG
            ShowStack();
#endif
*/
        }
    }
    
    if (sav_left_sw != TheLeftSwitch)
    {
        sav_left_sw = TheLeftSwitch;
        if (sav_left_sw)
        {
#ifdef UART_DEBUG
            ShowCANItems();
#endif
        }
    }
    
    if (sav_right_sw != TheRightSwitch)
    {
        sav_right_sw = TheRightSwitch;
        if (sav_right_sw)
        {
#ifdef UART_DEBUG
            ShowState();
#endif
        }
    }
//#endif  // CAN_DEVICE_LM3S8962
        
    if (sav_up_sw != TheUpSwitch)
    {
        sav_up_sw = TheUpSwitch;
        if (sav_up_sw)
        {
#ifdef CO_IO_DIGITAL_INPUTS
            ++ODE_Read_Digital_Input.BYTE[0];

#endif
#ifdef UART_DEBUG
            // ShowObjectDictionary();
#endif
        }
    }

    if (sav_down_sw != TheDownSwitch)
    {
        sav_down_sw = TheDownSwitch;
        if (sav_down_sw)
        {
#ifdef CO_IO_DIGITAL_INPUTS
            --ODE_Read_Digital_Input.BYTE[0];
#endif
        }
    }
    
    // display variables

//#ifdef CAN_DEVICE_LM3S8962

    RIT128x96x4Enable(1000000);

    buf = main_mpu6050();

    accelUn.bufAccel[0] = buf[0];
    accelUn.bufAccel[1] = buf[1];
    accelUn.bufAccel[2] = buf[2];

    gyroUn.bufGyro[0] = buf[4];
    gyroUn.bufGyro[1] = buf[5];
    gyroUn.bufGyro[2] = buf[6];

    quatUn.bufQuaternion[0] = buf[7];
    quatUn.bufQuaternion[1] = buf[8];
    quatUn.bufQuaternion[2] = buf[9];
    quatUn.bufQuaternion[3] = buf[10];

    tempUn.bufTemp = buf[3];

    if (tempUn.bufTemp != ODE_Read_Digital_Input.DWORD[0])
    {
        CO_DISABLE_TMR();

        RIT128x96x4StringDraw(ftoa(accelUn.bufAccel[0]), 50, 0, 15);
        RIT128x96x4StringDraw(ftoa(accelUn.bufAccel[1]), 50, 8, 15);
        RIT128x96x4StringDraw(ftoa(accelUn.bufAccel[2]), 50, 16, 15);
        //SysCtlDelay(100);
        RIT128x96x4StringDraw(ftoa(gyroUn.bufGyro[0]), 50, 24, 15);
        RIT128x96x4StringDraw(ftoa(gyroUn.bufGyro[1]), 50, 32, 15);
        RIT128x96x4StringDraw(ftoa(gyroUn.bufGyro[2]), 50, 40, 15);
        RIT128x96x4StringDraw(ftoa(quatUn.bufQuaternion[0]), 50, 48, 15);
        RIT128x96x4StringDraw(ftoa(quatUn.bufQuaternion[1]), 50, 56, 15);
        RIT128x96x4StringDraw(ftoa(quatUn.bufQuaternion[2]), 50, 64, 15);
        RIT128x96x4StringDraw(ftoa(quatUn.bufQuaternion[3]), 50, 72, 15);
        RIT128x96x4StringDraw(ftoa(tempUn.bufTemp), 50, 80, 15);



#ifdef UART_DEBUG
        UARTprintf("complete data receive from MPU6050\n");
        UARTprintf("Accel x = %s", ftoa(accelUn.bufAccel[0]));
        UARTprintf(", y = %s", ftoa(accelUn.bufAccel[1]));
        UARTprintf(", z = %s\n", ftoa(accelUn.bufAccel[2]));
        UARTprintf("Gyro x = %s", ftoa(gyroUn.bufGyro[0]));
        UARTprintf(", y = %s", ftoa(gyroUn.bufGyro[1]));
        UARTprintf(", z = %s\n", ftoa(gyroUn.bufGyro[2]));
        UARTprintf("Temperature x = %s\n", ftoa(tempUn.bufTemp));
#endif
        CO_ENABLE_TMR();
    }

    if (state != CO_NMToperatingState)
    {
#ifdef UART_DEBUG
        UARTprintf("CO_NMToperatingState = ");
#endif
        switch (CO_NMToperatingState)
        {
            case NMT_INITIALIZING:
                RIT128x96x4StringDraw("INITIALIZING   ", 40, 88, 15);
#ifdef UART_DEBUG
                UARTprintf("NMT_INITIALIZING");
#endif
                break;
            case NMT_PRE_OPERATIONAL:
                RIT128x96x4StringDraw("PRE-OPERATIONAL", 40, 88, 15);
#ifdef UART_DEBUG
                UARTprintf("NMT_PRE_OPERATIONAL");
#endif
                break;
            case NMT_OPERATIONAL:
                RIT128x96x4StringDraw("OPERATIONAL    ", 40, 88, 15);
#ifdef UART_DEBUG
                UARTprintf("NMT_OPERATIONAL");
#endif
                break;
            case NMT_STOPPED:
                RIT128x96x4StringDraw("STOPPED        ", 40, 88, 15);
#ifdef UART_DEBUG
                UARTprintf("NMT_STOPPED");
#endif
                break;
            default:
#ifdef UART_DEBUG
                UARTprintf("0x%02x", CO_NMToperatingState);
#endif
                break;
        }
#ifdef UART_DEBUG
        UARTprintf("\n");
#endif
        state = CO_NMToperatingState;
    }
//#endif  // CAN_DEVICE_LM3S8962
}

/*****************************************************************************//**
   User_Process1msIsr - 1 ms USER TIMER FUNCTION
   
   Function is executed every 1 ms. It is deterministic and has priority over
   mainline functions.

   Function of the following code (each cycle):
    - Read from hardware
    - If Operational state:
       - If all Monitored nodes are Operational:
          - Copy all Received PDOs to their mapped location
       - Prepare all Transmit PDOs from their mapped location (and send them in case Change Of State)
    - If Operational state of this or monitored nodes is lost, node outputs are switched off.
    - Write to hardware

   If TPDO is configured for Change of State transmission (Transmission_type >= 254), then consider next
   situation: On network startup one node is started first and send PDO of curent state, which will not
   change soon. Another node is started later and missed PDO from first node??? Solution might be
   Event timer.
*******************************************************************************/
void User_Process1msIsr(void)
{
    static unsigned char LastStateOperationalGradePrev = 0;
    unsigned char LastStateOperationalGrade = 0;

//#ifdef CAN_DEVICE_LM3S8962
    TheSelectSwitch = SELECT_SW_PRESSED();    
    TheLeftSwitch = LEFT_SW_PRESSED();
    TheRightSwitch = RIGHT_SW_PRESSED();
//#endif
    TheUpSwitch = UP_SW_PRESSED();
    TheDownSwitch = DOWN_SW_PRESSED();
    
    // Read from Hardware -------------------------------------------------------
    //CHANGE THIS LINE -> ODE_Read_Digital_Input.BYTE[0] = port_xxx
    //CHANGE THIS LINE -> ODE_Read_Digital_Input.BYTE[1] = port_xxx
    //CHANGE THIS LINE -> ODE_Read_Digital_Input.BYTE[2] = port_xxx
    //CHANGE THIS LINE -> ODE_Read_Digital_Input.BYTE[3] = port_xxx
#ifdef CO_IO_ANALOG_INPUTS                        
    //CHANGE THIS LINE -> ODE_Read_Analog_Input[0...7] = ...
#endif

    // PDO Communication
    if (CO_NMToperatingState == NMT_OPERATIONAL)
    {
        LastStateOperationalGrade++;

        // verify operating state of monitored nodes
#if CO_NO_CONS_HEARTBEAT > 0
        if (CO_HBcons_AllMonitoredOperational == NMT_OPERATIONAL) 
        {
#endif
            LastStateOperationalGrade++;

            // Read RPDOs -------------------------------------------------------------
            // Following code realizes Static RPDO Mapping
#if CO_NO_RPDO > 0
#ifdef CO_IO_DIGITAL_OUTPUTS
            if (CO_RPDO_New(0)) 
            {
                CO_DISABLE_CANRX_TMR();
                ODE_Write_Digital_Output.DWORD[0] = CO_RPDO(0).DWORD[0];
                CO_RPDO_New(0) = 0;
                CO_ENABLE_CANRX_TMR();
            }
#endif
#endif

#if CO_NO_RPDO > 1
#ifdef CO_IO_ANALOG_OUTPUTS
            if (CO_RPDO_New(1)) 
            {
                CO_DISABLE_CANRX_TMR();
                ODE_Write_Analog_Output[0] = CO_RPDO(1).WORD[0];
                ODE_Write_Analog_Output[1] = CO_RPDO(1).WORD[1];
                CO_RPDO_New(1) = 0;
                CO_ENABLE_CANRX_TMR();
            }
#endif
#endif

#if CO_NO_CONS_HEARTBEAT > 0
        } // end if(CO_HBcons_AllMonitoredOperational == NMT_OPERATIONAL)
#endif

        // Write TPDOs ------------------------------------------------------------
        // Following code realizes Static TPDO Mapping
        // Transmission is Synchronous or Change of State, depends on Transmission_type.
        // Inhibit timer and Periodic Event Timer can be used.
#if CO_NO_TPDO > 0
#endif

#if CO_NO_TPDO > 1
#ifdef CO_IO_DIGITAL_INPUTS
        if ((CO_TPDO_InhibitTimer[1] == 0) &&
            (CO_TPDO(1).DWORD[0] != ODE_Read_Digital_Input.DWORD[0])) 
        {
            CO_TPDO(1).DWORD[0] = tempUn.ul;

            if (ODE_TPDO_Parameter[1].Transmission_type >= 254)
            {
                CO_TPDOsend(1);
            }
        }
#elif defined CO_IO_ANALOG_INPUTS
        if ((CO_TPDO_InhibitTimer[1] == 0) && (
            (CO_TPDO(1).WORD[0] != ODE_Read_Analog_Input[0]) ||
            (CO_TPDO(1).WORD[1] != ODE_Read_Analog_Input[1]) ||
            (CO_TPDO(1).WORD[2] != ODE_Read_Analog_Input[2]) ||
            (CO_TPDO(1).WORD[3] != ODE_Read_Analog_Input[3]))) 
        {
            CO_TPDO(1).WORD[0] = ODE_Read_Analog_Input[0];
            CO_TPDO(1).WORD[1] = ODE_Read_Analog_Input[1];
            CO_TPDO(1).WORD[2] = ODE_Read_Analog_Input[2];
            CO_TPDO(1).WORD[3] = ODE_Read_Analog_Input[3];
            if (ODE_TPDO_Parameter[1].Transmission_type >= 254)
            {
                CO_TPDOsend(1);
            }
        }
#endif
#endif

    } // end if (CO_NMToperatingState == NMT_OPERATIONAL)

    if (LastStateOperationalGrade < LastStateOperationalGradePrev) // NMT_OPERATIONAL (this or monitored nodes) was just lost
    {
        SwitchOffNode();
    }
    LastStateOperationalGradePrev = LastStateOperationalGrade;

}

