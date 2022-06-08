

/**
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcomp.h"
#include "usblib/device/usbdcdc.h"
#include "utils/ustdlib.h"
#include "usb_structs.h"
#include "drivers/pinout.h"



#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


typedef struct led_t_ {
    uint32_t port_clock;
    uint32_t port_base;
    uint32_t port_pin;
    uint32_t time;
}led_t;


void task1(void *param){
    led_t *led = (led_t *)param;

    volatile float teste = 0.01;
    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(led->port_clock);

    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(led->port_clock))
    {
        vTaskDelay(100);
    }

    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(led->port_base, led->port_pin);

    while(1){
        // Turn on the LED.
        GPIOPinWrite(led->port_base, led->port_pin, led->port_pin);
        vTaskDelay(led->time);

        teste += 0.01;

        // Turn off the LED.
        GPIOPinWrite(led->port_base, led->port_pin, 0);
        vTaskDelay(led->time);
    }
}


volatile int count_leftover = 0;
void task_cpu_leftover(void *param){
    TickType_t last;
    TickType_t frequency = 10;
    int i = 0;
    int result = 0;

    last = xTaskGetTickCount();

    while(1){
        for (i=0; i<1024; i++){
            result = result ^ i;
        }
        xTaskDelayUntil(&last, frequency);
        count_leftover++;
    }
}

SemaphoreHandle_t sem_t3;

void task3(void *param){
    uint32_t time = (uint32_t)param;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC_UP);
    MAP_IntPrioritySet(INT_TIMER1A, 0xC0);
    ROM_IntEnable(INT_TIMER1A);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    TimerLoadSet(TIMER1_BASE, TIMER_BOTH, time * (120000000/20000));
    //TimerLoadSet(TIMER1_BASE, TIMER_BOTH, time );
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);

    sem_t3 = xSemaphoreCreateBinary();

    while(1){
        xSemaphoreTake(sem_t3, portMAX_DELAY);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);

        xSemaphoreTake(sem_t3, portMAX_DELAY);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
    }
}

void timer1_A_IRQ(void){
    portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    if (sem_t3 != NULL){
        xSemaphoreGiveFromISR(sem_t3, &pxHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}


#define UART_CARACTERE 1
#define UART_QUEUE     2
#define UART_STRING UART_QUEUE


// Declares a queue structure for the UART
QueueHandle_t qUART;
#if UART_STRING == UART_QUEUE
QueueHandle_t qUART_Tx;
volatile uint32_t isstring = 0;
#endif

// Declares a semaphore structure for the UART
SemaphoreHandle_t sUART;

// Declares a mutex structure for the UART
SemaphoreHandle_t mutexTx;


portBASE_TYPE UARTGetChar(char *data, TickType_t timeout){
    return xQueueReceive(qUART, data, timeout);
}


void UARTPutChar(uint32_t ui32Base, char ucData){
    if (mutexTx != NULL){
        if (xSemaphoreTake(mutexTx, portMAX_DELAY) == pdTRUE){
            //
            // Send the char.
            HWREG(ui32Base + UART_O_DR) = ucData;

            //
            // Wait until space is available.
            MAP_UARTIntEnable(UART0_BASE, UART_INT_TX);

            // Wait indefinitely for a UART interrupt
            xSemaphoreTake(sUART, portMAX_DELAY);

            xSemaphoreGive(mutexTx);
        }
    }
}


#if UART_STRING == UART_CARACTERE
void UARTPutString(uint32_t ui32Base, char *string){
    if (mutexTx != NULL){
        if (xSemaphoreTake(mutexTx, portMAX_DELAY) == pdTRUE){
            while(*string){
                //
                // Send the char.
                HWREG(ui32Base + UART_O_DR) = *string;

                MAP_UARTIntEnable(UART0_BASE, UART_INT_TX);

                // Wait indefinitely for a UART interrupt
                xSemaphoreTake(sUART, portMAX_DELAY);

                string++;
            }

            xSemaphoreGive(mutexTx);
        }
    }
}
#else
void UARTPutString(uint32_t ui32Base, char *string){
    char data;
    if (mutexTx != NULL){
        if (xSemaphoreTake(mutexTx, portMAX_DELAY) == pdTRUE){
            isstring = 1;
            while(*string){
                xQueueSendToBack(qUART_Tx, string, portMAX_DELAY);

                string++;
            }

            // Send the char.
            xQueueReceive(qUART_Tx, &data, portMAX_DELAY);
            HWREG(ui32Base + UART_O_DR) = data;

            MAP_UARTIntEnable(UART0_BASE, UART_INT_TX);

            // Wait indefinitely the finish of the string transmission by the UART port
            xSemaphoreTake(sUART, portMAX_DELAY);

            xSemaphoreGive(mutexTx);
        }
    }
}
#endif

void UARTIntHandler(void){
    uint32_t ui32Status;
    signed portBASE_TYPE pxHigherPriorityTaskWokenRX = pdFALSE;
    signed portBASE_TYPE pxHigherPriorityTaskWokenTX = pdFALSE;
    char data;

    #if UART_STRING == UART_QUEUE
    BaseType_t ret;
    #endif

    //
    // Get the interrrupt status.
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    UARTIntClear(UART0_BASE, ui32Status);

    if ((ui32Status&UART_INT_RX) == UART_INT_RX){
        //
        // Loop while there are characters in the receive FIFO.
        while(MAP_UARTCharsAvail(UART0_BASE)){
            //
            // Read the next character from the UART and write it back to the UART.
            data = (char)MAP_UARTCharGetNonBlocking(UART0_BASE);
            xQueueSendToBackFromISR(qUART, &data, &pxHigherPriorityTaskWokenRX);
        }
    }

    if ((ui32Status&UART_INT_TX) == UART_INT_TX){
        #if UART_STRING == UART_QUEUE
        if (isstring){
            // read the char to be sent.
            ret = xQueueReceiveFromISR(qUART_Tx, &data, NULL);

            if (ret){
                // Send the char
                HWREG(UART0_BASE + UART_O_DR) = data;
            }else{
                // There is no more char to be send in this string
                isstring = 0;
                MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);

                // Wake up the task that is sending the string
                xSemaphoreGiveFromISR(sUART, &pxHigherPriorityTaskWokenTX);
            }
        }else{
            MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);

            // Call the task that is sending the char
            xSemaphoreGiveFromISR(sUART, &pxHigherPriorityTaskWokenTX);
        }
        #else
        MAP_UARTIntDisable(UART0_BASE, UART_INT_TX);

        // Call the keyboard analysis task
        xSemaphoreGiveFromISR(sUART, &pxHigherPriorityTaskWokenTX);
        #endif
    }

    if ((pxHigherPriorityTaskWokenRX == pdTRUE) || (pxHigherPriorityTaskWokenTX == pdTRUE)){
        portYIELD();
    }
}


void task_serial2(void *param){
    volatile int count = 0;
    vTaskDelay(500);
    while(1){
        UARTPutString(UART0_BASE,"Tarefa intrusa!\n\r");
        vTaskDelay(1000);
    }
}

void task_serial(void *param){
    char *teste_serial = "Teste de porta serial!\n\r";
    volatile int count = 0;
#if 0
    // Enable the peripherals used by this example.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Set GPIO A0 and A1 as UART pins.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    MAP_UARTConfigSetExpClk(UART0_BASE,  120000000, 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

    volatile TickType_t start = xTaskGetTickCount();

    while(1){
        char *string = teste_serial;
        while(*string){
            //UARTCharPut(UART0_BASE, *string);
            UARTCharPutNonBlocking(UART0_BASE, *string);
            while (UARTBusy(UART0_BASE)){
                portYIELD();
            }
            string++;
        }
        count++;
        volatile TickType_t stop = xTaskGetTickCount();
        if ((stop - start) >= 1000){
            start = stop;
        }
    }
#else
    sUART = xSemaphoreCreateBinary();

    if( sUART == NULL ){
        /* There was insufficient FreeRTOS heap available for the semaphore to
        be created successfully. */
        vTaskSuspend(NULL);
    }
    else
    {
        mutexTx = xSemaphoreCreateMutex();
        if( mutexTx == NULL ){
            /* There was insufficient FreeRTOS heap available for the semaphore to
            be created successfully. */
            vSemaphoreDelete( sUART);
            vTaskSuspend(NULL);
        }else{
            qUART = xQueueCreate(128, sizeof(char));
            if( qUART == NULL ){
                /* There was insufficient FreeRTOS heap available for the queue to
                be created successfully. */
                vSemaphoreDelete( sUART);
                vSemaphoreDelete( mutexTx);
                vTaskSuspend(NULL);
            }else
            {
                   #if UART_STRING == UART_QUEUE
                   qUART_Tx = xQueueCreate(128, sizeof(char));
                   if( qUART_Tx == NULL ){
                       /* There was insufficient FreeRTOS heap available for the queue to
                       be created successfully. */
                       vSemaphoreDelete( sUART);
                       vSemaphoreDelete( mutexTx);
                       vQueueDelete( qUART);
                       vTaskSuspend(NULL);
                   }
                   #endif
                   //
                   // Enable the peripherals used by this example.
                   MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
                   MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

                   //
                   // Set GPIO A0 and A1 as UART pins.
                   MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
                   MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
                   MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

                   //
                   // Configure the UART for 115,200, 8-N-1 operation.
                   MAP_UARTConfigSetExpClk(UART0_BASE,  120000000, 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

                   MAP_UARTFIFODisable(UART0_BASE);
                   //
                   // Enable the UART interrupt.
                   MAP_IntPrioritySet(INT_UART0, 0xC0);
                   MAP_IntEnable(INT_UART0);
                   MAP_UARTIntEnable(UART0_BASE, UART_INT_RX);
            }
        }
    }

    /*
    volatile TickType_t start = xTaskGetTickCount();

    while(1){
        UARTPutString(UART0_BASE,teste_serial);
        count++;
        volatile TickType_t stop = xTaskGetTickCount();
        if ((stop - start) >= 1000){
            start = stop;
        }
    }
    */

    // Limpa a tela
    UARTPutString(UART0_BASE,"\033[2J\033[H");

    // Avisa que o sistema iniciou!
    UARTPutString(UART0_BASE,"FreeRTOS started!\n\r");

    char data;
    while(1)
    {
        xQueueReceive(qUART, &data, portMAX_DELAY);
        if (data != 13){
            UARTPutChar(UART0_BASE,data);
        }else{
            UARTPutString(UART0_BASE,"\n\r");
        }
    }
#endif
}

TaskHandle_t task1_handle;
TaskHandle_t task2_handle;


//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;


//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;
static bool g_bSendingBreak = false;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile bool g_bUSBConfigured = false;

//*****************************************************************************
//
// Internal function prototypes.
//
//*****************************************************************************
static void USBUARTPrimeTransmit();
static void SetControlLineState(uint16_t ui16State);
static bool SetLineCoding(tLineCoding *psLineCoding);
static void GetLineCoding(tLineCoding *psLineCoding);
static void SendBreak(bool bSend);

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1)
    {
    }
}
#endif



//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the USB UART's transmit FIFO.
//
//*****************************************************************************
static void USBUARTPrimeTransmit(void)
{
    uint32_t ui32Read;
    uint32_t read_bytes;
    uint8_t ui8Char;

    //
    // If we are currently sending a break condition, don't receive any
    // more data. We will resume transmission once the break is turned off.
    //
    if(g_bSendingBreak)
    {
        return;
    }

    //
    // If there is space in the UART FIFO, try to read some characters
    // from the receive buffer to fill it again.
    //
    //while(ROM_UARTSpaceAvail(ui32Base))
    //{
        //
        // Get a character from the buffer.
        //
        ui32Read = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ui8Char, 1);

        //
        // Did we get a character?
        //
        if(ui32Read)
        {
            //
            // Place the character in the UART transmit FIFO.
            //
            //ROM_UARTCharPutNonBlocking(ui32Base, ui8Char);

            read_bytes = 0;
            do
            {
                //(void)OSQueuePost(USB, ucChar[read_bytes]);
                read_bytes++;
                ui32Read--;
            }while(ui32Read);

            //
            // Update our count of bytes transmitted via the UART.
            //
            //g_ui32UARTTxCount++;
        }
        else
        {
            //
            // We ran out of characters so exit the function.
            //
            return;
        }
    //}
}





//*****************************************************************************
//
// Set the state of the RS232 RTS and DTR signals.
//
//*****************************************************************************
static void
SetControlLineState(uint16_t ui16State)
{
    //
    // TODO: If configured with GPIOs controlling the handshake lines,
    // set them appropriately depending upon the flags passed in the wValue
    // field of the request structure passed.
    //
}

//*****************************************************************************
//
// Set the communication parameters to use on the UART.
//
//*****************************************************************************
static bool
SetLineCoding(tLineCoding *psLineCoding)
{
    uint32_t ui32Config;
    bool bRetcode;

    //
    // Assume everything is OK until we detect any problem.
    //
    bRetcode = true;

    //
    // Word length.  For invalid values, the default is to set 8 bits per
    // character and return an error.
    //
    switch(psLineCoding->ui8Databits)
    {
        case 5:
        {
            ui32Config = UART_CONFIG_WLEN_5;
            break;
        }

        case 6:
        {
            ui32Config = UART_CONFIG_WLEN_6;
            break;
        }

        case 7:
        {
            ui32Config = UART_CONFIG_WLEN_7;
            break;
        }

        case 8:
        {
            ui32Config = UART_CONFIG_WLEN_8;
            break;
        }

        default:
        {
            ui32Config = UART_CONFIG_WLEN_8;
            bRetcode = false;
            break;
        }
    }

    //
    // Parity.  For any invalid values, we set no parity and return an error.
    //
    switch(psLineCoding->ui8Parity)
    {
        case USB_CDC_PARITY_NONE:
        {
            ui32Config |= UART_CONFIG_PAR_NONE;
            break;
        }

        case USB_CDC_PARITY_ODD:
        {
            ui32Config |= UART_CONFIG_PAR_ODD;
            break;
        }

        case USB_CDC_PARITY_EVEN:
        {
            ui32Config |= UART_CONFIG_PAR_EVEN;
            break;
        }

        case USB_CDC_PARITY_MARK:
        {
            ui32Config |= UART_CONFIG_PAR_ONE;
            break;
        }

        case USB_CDC_PARITY_SPACE:
        {
            ui32Config |= UART_CONFIG_PAR_ZERO;
            break;
        }

        default:
        {
            ui32Config |= UART_CONFIG_PAR_NONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Stop bits.  Our hardware only supports 1 or 2 stop bits whereas CDC
    // allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
    // invalid or unsupported value of ui8Stop, we set up for 1 stop bit but
    // return an error in case the caller needs to Stall or otherwise report
    // this back to the host.
    //
    switch(psLineCoding->ui8Stop)
    {
        //
        // One stop bit requested.
        //
        case USB_CDC_STOP_BITS_1:
        {
            ui32Config |= UART_CONFIG_STOP_ONE;
            break;
        }

        //
        // Two stop bits requested.
        //
        case USB_CDC_STOP_BITS_2:
        {
            ui32Config |= UART_CONFIG_STOP_TWO;
            break;
        }

        //
        // Other cases are either invalid values of ui8Stop or values that we
        // cannot support so set 1 stop bit but return an error.
        //
        default:
        {
            ui32Config |= UART_CONFIG_STOP_ONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Set the UART mode appropriately.
    //
    //ROM_UARTConfigSetExpClk(USB_UART_BASE, ROM_SysCtlClockGet(),
    //                        psLineCoding->ui32Rate, ui32Config);

    //
    // Let the caller know if we had a problem or not.
    //
    return(bRetcode);
}

//*****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//*****************************************************************************
#define DEFAULT_BIT_RATE        115200
#define DEFAULT_UART_CONFIG     (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE)

static void GetLineCoding(tLineCoding *psLineCoding)
{
    uint32_t ui32Config;
    uint32_t ui32Rate;

    //
    // Get the current line coding set in the UART.
    //
    //ROM_UARTConfigGetExpClk(USB_UART_BASE, ROM_SysCtlClockGet(), &ui32Rate,
    //                        &ui32Config);

    //
    // Get the current line coding set in the UART.
    //
    ui32Config = DEFAULT_UART_CONFIG;
    psLineCoding->ui32Rate = DEFAULT_BIT_RATE;

    //psLineCoding->ui32Rate = ui32Rate;

    //
    // Translate the configuration word length field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_WLEN_MASK)
    {
        case UART_CONFIG_WLEN_8:
        {
            psLineCoding->ui8Databits = 8;
            break;
        }

        case UART_CONFIG_WLEN_7:
        {
            psLineCoding->ui8Databits = 7;
            break;
        }

        case UART_CONFIG_WLEN_6:
        {
            psLineCoding->ui8Databits = 6;
            break;
        }

        case UART_CONFIG_WLEN_5:
        {
            psLineCoding->ui8Databits = 5;
            break;
        }
    }

    //
    // Translate the configuration parity field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_PAR_MASK)
    {
        case UART_CONFIG_PAR_NONE:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_NONE;
            break;
        }

        case UART_CONFIG_PAR_ODD:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_ODD;
            break;
        }

        case UART_CONFIG_PAR_EVEN:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_EVEN;
            break;
        }

        case UART_CONFIG_PAR_ONE:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_MARK;
            break;
        }

        case UART_CONFIG_PAR_ZERO:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_SPACE;
            break;
        }
    }

    //
    // Translate the configuration stop bits field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_STOP_MASK)
    {
        case UART_CONFIG_STOP_ONE:
        {
            psLineCoding->ui8Stop = USB_CDC_STOP_BITS_1;
            break;
        }

        case UART_CONFIG_STOP_TWO:
        {
            psLineCoding->ui8Stop = USB_CDC_STOP_BITS_2;
            break;
        }
    }
}

//*****************************************************************************
//
// This function sets or clears a break condition on the redirected UART RX
// line.  A break is started when the function is called with \e bSend set to
// \b true and persists until the function is called again with \e bSend set
// to \b false.
//
//*****************************************************************************
static void SendBreak(bool bSend)
{
    //
    // Are we being asked to start or stop the break condition?
    //
    if(!bSend)
    {
        //
        // Remove the break condition on the line.
        //
        //ROM_UARTBreakCtl(USB_UART_BASE, false);
        g_bSendingBreak = false;
    }
    else
    {
        //
        // Start sending a break condition on the line.
        //
        //ROM_UARTBreakCtl(USB_UART_BASE, true);
        g_bSendingBreak = true;
    }
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    uint32_t ui32IntsOff;
    UBaseType_t uxSavedInterruptStatus;

    //
    // Which event are we being asked to process?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            //
            // Tell the main loop to update the display.
            //
            uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
            g_pcStatus = "Connected";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
            g_pcStatus = "Disconnected";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
            SetLineCoding(pvMsgData);
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            SetControlLineState((uint16_t)ui32MsgValue);
            break;

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
            SendBreak(true);
            break;

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
            SendBreak(false);
            break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    uint32_t ui32Count;

    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //
            // Feed some characters into the UART TX FIFO and enable the
            // interrupt so we are told when there is more space.
            //
            USBUARTPrimeTransmit();
            //ROM_UARTIntEnable(USB_UART_BASE, UART_INT_TX);
            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            //
            // Get the number of bytes in the buffer and add 1 if some data
            // still has to clear the transmitter.
            //
            //ui32Count = ROM_UARTBusy(USB_UART_BASE) ? 1 : 0;
            //return(ui32Count);
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }

    return(0);
}


void usb_task(void *param){
    (void)param;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);

    //
    // Not configured initially.
    //
    g_bUSBConfigured = false;

    //
    // Configure the device pins.
    //
    PinoutSet(false, true);

    //
    // Initialize the transmit and receive buffers.
    //
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);


    //
    // Tell the USB library the CPU clock and the PLL frequency.  This is a
    // new requirement for TM4C129 devices.
    //
    uint32_t ui32PLLRate;
    SysCtlVCOGet(SYSCTL_XTAL_25MHZ, &ui32PLLRate);
    USBDCDFeatureSet(0, USBLIB_FEATURE_CPUCLK, &g_ui32SysClock);
    USBDCDFeatureSet(0, USBLIB_FEATURE_USBPLL, &ui32PLLRate);

    MAP_IntPrioritySet(INT_USB0, 0xC0);
    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDCDCInit(0, &g_sCDCDevice);

    while(1){
        USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)"Teste\r\n", 7);
        vTaskDelay(1000);

    }
}

int main(void)
{
    // Make sure the main oscillator is enabled because this is required by
    // the PHY.  The system must have a 25MHz crystal attached to the OSC
    // pins. The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
    // frequency is 10MHz or higher.
    MAP_SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

    // Run from the PLL at 120 MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);

    // Enable stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();

    static led_t led_t1 = {SYSCTL_PERIPH_GPION, GPIO_PORTN_BASE, GPIO_PIN_0, 500};
    xTaskCreate(task1, "Tarefa 1", 256, &led_t1, 10, &task1_handle);

    static led_t led_t2 = {SYSCTL_PERIPH_GPION, GPIO_PORTN_BASE, GPIO_PIN_1, 1000};
    xTaskCreate(task1, "Tarefa 2", 256, &led_t2, 9, &task2_handle);

    uint32_t *time = (uint32_t *)10;
    xTaskCreate(task3, "Tarefa 3", 256, time, 12, NULL);

    xTaskCreate(task_serial, "Tarefa Serial", 256, NULL, 11, NULL);
    xTaskCreate(task_serial2, "Tarefa Serial 2", 256, NULL, 11, NULL);

    xTaskCreate(usb_task, "Tarefa USB", 512, NULL, 8, NULL);

    xTaskCreate(task_cpu_leftover, "Tarefa sobra cpu", 256, NULL, 1, NULL);



    /* Start the scheduler. */
    vTaskStartScheduler();


    return 0;
}



