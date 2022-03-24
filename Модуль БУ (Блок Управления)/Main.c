// CAN board is specified in Makefile
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
    // If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    // a workaround to allow the PLL to operate reliably.
    //
    if (REVISION_IS_A2)     // needs #include "inc/hw_sysctl.h"
    {
        SysCtlLDOSet(SYSCTL_LDO_2_75V);
    }

    //
    // Set the clocking to run from the PLL.
    //
    SysCtlClockSet(CO_OSCILLATOR_FREQ | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);

    //
    // Disable interrupts to the processor.
    //
    IntMasterDisable();

#ifdef CAN_DEVICE_LM3S2110
    //
    // Enable the pull-ups on the JTAG signals. 
    // (This appears to be needed for reliable operation!?)
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_JTAG_PINS);
    GPIOPadConfigSet(JTAG_BASE,
                     JTAG_TCK | JTAG_TMS | JTAG_TDI | JTAG_TDO,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
#endif  // CAN_DEVICE_LM3S2110

#ifdef UART_DEBUG
    //
    // Set up the serial console to use for displaying messages.
    //
    InitConsole();
    UARTprintf("\n\nCANopenNode Tutorial Command\n");
    UARTprintf("============================\n");
#endif  // UART_DEBUG
    
    //
    // Initialize the OLED display.
    //
    RIT128x96x4Init(1000000);

    //
    // Hello!
    //
    RIT128x96x4StringDraw(" ", 8, 0, 0);      // remove bogus pixel on Board #1
    RIT128x96x4StringDraw("CANopenNode", 20, 20, 15);
    RIT128x96x4StringDraw("Tutorial Command", 20, 32, 15);
    RIT128x96x4StringDraw("Temperature = ", 0, 48, 15);
    RIT128x96x4StringDraw("Cooler: ", 0, 58, 15);
    RIT128x96x4StringDraw("Heater: ", 0, 66, 15);
    RIT128x96x4StringDraw("State: ", 0, 76, 15);

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

    while (1) 
    {
        CO_ProcessMain();
        User_ProcessMain();
    }
}

Модуль БУ – BlockOfControl.c


// CAN board is specified in Makefile
#include <stdlib.h>
#include "EK-LM3S8962-HW-Main.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "drivers/rit128x96x4.h"
#include "CANopen.h"
#include "debug_info.h"

#define RemoteTemperature           CO_RPDO(0).DWORD[0]

#define NMT_MASTER                  CO_TXCAN_USER

    #define DigOut_COOLER_IND_INIT()    PC6_PIN_INIT()
    #define DigOut_COOLER_IND_ON()      TURN_ON_PC6()
    #define DigOut_COOLER_IND_OFF()     TURN_OFF_PC6()
    #define DigOut_HEATER_IND_INIT()    PD3_PIN_INIT()
    #define DigOut_HEATER_IND_ON()      TURN_ON_PD3()
    #define DigOut_HEATER_IND_OFF()     TURN_OFF_PD3()
    #define DigInp_Button_INIT()        SELECT_SW_INIT()
    #define DigInp_Button()             SELECT_SW_PRESSED()

#define DigOut_COOLER_IND(i)        if (i) { DigOut_COOLER_IND_ON(); } else DigOut_COOLER_IND_OFF()
#define DigOut_HEATER_IND(i)        if (i) { DigOut_HEATER_IND_ON(); } else DigOut_HEATER_IND_OFF()

static int TheSelectSwitch = 0;
static int TheLeftSwitch = 0;
static int TheRightSwitch = 0;

static int TheUpSwitch = 0;
static int TheDownSwitch = 0;

static char * Temperature = 0;

union
{
	float bufTemp;
	unsigned long ul;
	unsigned char uc[32];
} bufUn;

char cbuf[100];
char* ftoa(float f)
{
	int pos=0,ix,dp,num;
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
   User_Init - USER INITIALIZATION OF NODE
   
   Function is called after start of program.
*******************************************************************************/
void User_Init(void)
{
    ODE_EEPROM.PowerOnCounter++;

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
    // Prepare variables for custom made NMT master message.
    // arguments to CO_IDENT_WRITE(CAN_ID_11bit, RTRbit/*usually 0*/)
    CO_TXCAN[NMT_MASTER].Ident.WORD[0] = CO_IDENT_WRITE(0, 0);
    CO_TXCAN[NMT_MASTER].NoOfBytes = 2;
    CO_TXCAN[NMT_MASTER].NewMsg = 0;
    CO_TXCAN[NMT_MASTER].Inhibit = 0;
}

#ifdef UART_DEBUG
//! Output temperature info.
void ShowTemps(void)
{   
    UARTprintf("\nRemoteTemperature = %d\n", RemoteTemperature);
}
#endif  // UART_DEBUG

/*****************************************************************************//**
   User_ProcessMain - USER PROCESS MAINLINE
   
   This function is cyclically called from main(). It is a non blocking function.
   It is asynchronous. This is for longer and time consuming code.
*******************************************************************************/
void User_ProcessMain(void)
{
    static int sav_up_sw = 0;
    static int sav_down_sw = 0;
    static int sav_select_sw = 0;
    static int sav_left_sw = 0;
    static int sav_right_sw = 0;
    char tempString[5];
    static uint16_t saveTemp = 9999;
    static unsigned char status = 99;
    static unsigned char state = 99;

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
        
    if (sav_up_sw != TheUpSwitch)
    {
        sav_up_sw = TheUpSwitch;
        if (sav_up_sw)
        {
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
#ifdef UART_DEBUG
            ShowTemps();
#endif
        }
    }
    
    // display variables
    //DigOut_COOLER_IND(STATUS_COOLER);
    //DigOut_HEATER_IND(STATUS_HEATER);

    if (saveTemp != bufUn.bufTemp)
    {
        CO_DISABLE_TMR();
        saveTemp = bufUn.bufTemp;
        CO_ENABLE_TMR();
        RIT128x96x4StringDraw(ftoa(bufUn.bufTemp), 86, 48, 15);
#ifdef UART_DEBUG
        UARTprintf("Temperature = %s\n", tempString);
#endif
    }


    if (state != CO_NMToperatingState)
    {
#ifdef UART_DEBUG
        UARTprintf("CO_NMToperatingState = ");
#endif
        switch (CO_NMToperatingState)
        {
            case NMT_INITIALIZING:
                RIT128x96x4StringDraw("INITIALIZING   ", 40, 76, 15);
#ifdef UART_DEBUG
                UARTprintf("NMT_INITIALIZING");
#endif
                break;
            case NMT_PRE_OPERATIONAL:
                RIT128x96x4StringDraw("PRE-OPERATIONAL", 40, 76, 15);
#ifdef UART_DEBUG
                UARTprintf("NMT_PRE_OPERATIONAL");
#endif
                break;
            case NMT_OPERATIONAL:
                RIT128x96x4StringDraw("OPERATIONAL    ", 40, 76, 15);
#ifdef UART_DEBUG
                UARTprintf("NMT_OPERATIONAL");
#endif
                break;
            case NMT_STOPPED:
                RIT128x96x4StringDraw("STOPPED        ", 40, 76, 15);
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
}

/*****************************************************************************//**
   User_Process1msIsr - 1 ms USER TIMER FUNCTION
   
   Function is executed every 1 ms. It is deterministic and has priority over
   mainline functions.
*******************************************************************************/
void User_Process1msIsr(void)
{
    static unsigned int DeboucigTimer = 0;
    
    TheSelectSwitch = SELECT_SW_PRESSED();    
    TheLeftSwitch = LEFT_SW_PRESSED();
    TheRightSwitch = RIGHT_SW_PRESSED();
    TheUpSwitch = UP_SW_PRESSED();
    TheDownSwitch = DOWN_SW_PRESSED();
    
    if (DigInp_Button())
    {
        DeboucigTimer++;
    }
    else
    {
        DeboucigTimer = 0;
    }

    switch (DeboucigTimer)
    {
        case 1000:  // button is pressed for one seccond
            if (CO_NMToperatingState == NMT_OPERATIONAL)
            {
                CO_TXCAN[NMT_MASTER].Data.BYTE[0] = NMT_ENTER_PRE_OPERATIONAL;
                CO_NMToperatingState = NMT_PRE_OPERATIONAL;
            }
            else
            {
                CO_TXCAN[NMT_MASTER].Data.BYTE[0] = NMT_ENTER_OPERATIONAL;
                CO_NMToperatingState = NMT_OPERATIONAL;
            }
            CO_TXCAN[NMT_MASTER].Data.BYTE[1] = 0;  // all nodes
            CO_TXCANsend(NMT_MASTER);
            break;
        case 5000:  // button is pressed for 5 seconds
            // reset all remote nodes
            CO_TXCAN[NMT_MASTER].Data.BYTE[0] = NMT_RESET_NODE;
            CO_TXCAN[NMT_MASTER].Data.BYTE[1] = 0;  // all nodes
            CO_TXCANsend(NMT_MASTER);
            break;
        case 5010: // button is pressed for 5 seconds + 10 milliseconds
            CO_Reset();     // reset this node
            break;
    }

    bufUn.ul = RemoteTemperature;
}
