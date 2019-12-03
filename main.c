#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <stdio.h>
#include <string.h>
#include "driverlib/rtc.h"



/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result

int travel_distance = 0;
char travel_dist_str[6] = {0};
int done_writing_dist = 0;
int lcd_pos[6] = {4, 6, 8, 10, 2, 18};
char speed_str[6] = {0};
volatile int elapsed_seconds = 0;
char rtccnt_str[6];
int distance_travelled[2] = {0, 0};
int avg_dist = 0;


#pragma vector=RTC_VECTOR
__interrupt void RTC(void)
{
    ++elapsed_seconds;
    RTC_clearInterrupt(RTC_ADDRESS, RTC_OVERFLOW_INTERRUPT);

}

void main(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN7);
    clearLCD();
    // Initializations
    __disable_interrupt();
    WDT_A_hold(WDT_A_BASE);
    Init_GPIO();
    //Init_PWM();
    Init_ADC();
    Init_Clock();
    Init_UART();
    Init_LCD();
    PMM_unlockLPM5();

    __enable_interrupt();

    //Timer_A_outputPWM(TIMER_A0_BASE, &param);
    stop_motor();

    while (!done_writing_dist)
    {
       get_keypad_input();
       displayShowText(travel_dist_str);
       if(travel_distance > 999) {
           done_writing_dist = 1;
           break;
       }
    }
    //travel_distance = 1000;
    displayScrollText("DISTANCE HAS BEEN SET");
    displayShowText(travel_dist_str);
    clearLCD();
    RTC_init(RTC_ADDRESS, 1024 , RTC_CLOCKPREDIVIDER_1024);
    RTC_start(RTC_ADDRESS, RTC_CLOCKSOURCE_SMCLK);
    RTC_enableInterrupt(RTC_ADDRESS, RTC_OVERFLOW_INTERRUPT);
    while (avg_dist < travel_distance) {
        led_selector();
        get_direction();
        compute_revolutions();
        display_speed();
    }
    enable_red_led();
    stop_motor();
    displayScrollText("DESTINATION REACHED");

}

void led_selector() {
    if (avg_dist < travel_distance * 0.33) {
        enable_green_led();
    } else if (avg_dist < travel_distance * 0.66) {
        enable_yellow_led();
    } else {
        enable_orange_led();
    }
}

void enable_green_led() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);

}

void enable_yellow_led() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
}

void enable_orange_led() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
}

void enable_red_led() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
}

void check_forward() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
        go_forward();
    }
}

void check_backward() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
        go_backward();
    }
}

void check_left() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1) {
        stop_motor();
        go_left();
    }
}

void check_right() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == 1) {
        stop_motor();
        go_right();
    }
}

void check_stop() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
        stop_motor();
    }
}

void get_direction() {

    check_forward();
    check_backward();
    check_left();
    check_right();
    check_stop();
}

void display_speed() {
    sprintf(speed_str, "%d", avg_dist / elapsed_seconds);
    displayShowText(speed_str);
}

void stop_motor() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
}

void go_forward() {
    //wheels turn CW
//    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
}

void go_backward() {
    //displayShowText("BWD");
    //wheels turn CCW
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
//    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
}

void go_left() {
   // displayShowText("LFT");
    //left wheel turns CW
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
//    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
}

void go_right() {
    //displayShowText("RGT");
    //right wheel turns CW
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
//    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);

}

void compute_revolutions() {

    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0)
    {
    }

    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN0) == 0)
    {
        distance_travelled[1] += (WHEEL_DIAMETER * PI);
    }

    if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 0)
    {
        distance_travelled[0] += (WHEEL_DIAMETER * PI);
       //showHex(4);
    }
    avg_dist=(distance_travelled[0] + distance_travelled[1]) / 2;
}

void check_row_4() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
        travel_distance *= 10;
    }
    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1) {

    }
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == 1) {
        done_writing_dist = 1;
    }
    return;
}

void check_row_3() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
        travel_distance = travel_distance * 10 + 8;
    }
    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1) {
        travel_distance = travel_distance * 10 + 7;
    }
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == 1) {
        travel_distance = travel_distance * 10 + 9;
    }
    return;
}

void check_row_2() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
        travel_distance = travel_distance*10 + 5;
    }
    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1) {
        travel_distance = travel_distance*10 + 4;
    }
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == 1) {
        travel_distance = travel_distance*10 + 6;
    }

    return;
}

void check_row_1() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) {
        travel_distance = travel_distance*10 + 2;
    }
    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1) {
        travel_distance = travel_distance*10 + 1;
    }
    if(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) == 1) {
        travel_distance = travel_distance*10 + 3;
    }
    return;
}

void get_keypad_input() {
    check_row_1();
    check_row_2();
    check_row_3();
    check_row_4();
    sprintf(travel_dist_str, "%d", travel_distance);
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN0);

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN3);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN7);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
//    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
