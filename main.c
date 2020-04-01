#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/cpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"

#define COLOR_BLACK     0
#define COLOR_BLUE      1
#define COLOR_GREEN     2
#define COLOR_CYAN      3
#define COLOR_RED       4
#define COLOR_MAGENTA   5
#define COLOR_YELLOW    6
#define COLOR_WHITE     7

#define RED_DMA_CHAN    20
#define GREEN_DMA_CHAN  21
#define BLUE_DMA_CHAN   14

// Allocate the uDMA channel control table.  This one is a
// full table for all modes and channels.
// NOTE: This table must be 1024-byte aligned.

uint8_t dma_ctl_tbl[1024] __attribute__ ((aligned (1024)));

uint8_t bricks[12] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint16_t red_pix_data[] =   { 0x0078, 0x0000, 0x0000, 0x0000, 0x0000, 0x0F00, 0x0000 };
uint16_t green_pix_data[] = { 0x0780, 0x0000, 0x0000, 0x0000, 0x0000, 0xF000, 0x0000 };
uint16_t blue_pix_data[] =  { 0x7800, 0x0000, 0x0000, 0x0000, 0x000F, 0x0000, 0x0000 };

// Line / row on screen (in pixels)
int line = 0;

// Set upon entering vblank
volatile bool vblank = false;
volatile int bat_x = 60;
volatile int ball_x = 70;
volatile int ball_y = 507;

int ball_x_vel = 0;
int ball_y_vel = 0;

void hblank_isr(void);
void vblank_isr(void);
void pixel_data(void);

void hblank_isr(void)
{
    int int_stat = TimerIntStatus(TIMER0_BASE, true);
    TimerIntClear(TIMER0_BASE, int_stat);

    if (!(int_stat & TIMER_TIMB_TIMEOUT)) {

        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
        line++;

        if (line == 600) {
            vblank = true;
        }

        int i;
        int brick_color = COLOR_BLACK;

        if (line > 0x100) {
            brick_color = COLOR_BLACK;
        } else if (line > 0x0E0) {
            brick_color = COLOR_GREEN;
        } else if (line > 0x0C0) {
            brick_color = COLOR_CYAN;
        } else if (line > 0x0A0) {
            brick_color = COLOR_BLUE;
        } else if (line > 0x080) {
            brick_color = COLOR_YELLOW;
        } else if (line > 0x060) {
            brick_color = COLOR_MAGENTA;
        } else if (line > 0x040) {
            brick_color = COLOR_RED;
        } else {
            brick_color = COLOR_BLACK;
        }

        if ((line & 0x00F) < 4) {
            brick_color = COLOR_BLACK;
        }

        uint8_t bricks_byte = (line >= 0x40 && line < 0x100) ? bricks[(line - 0x40) >> 4] : 0x00;

        // Left wall
        red_pix_data[0] = 0x0078;
        green_pix_data[0] = 0x0780;
        blue_pix_data[0] = 0x7800;

        // Bricks
        for (i = 0; i < 4; i++) {

            uint16_t brick_left = (bricks_byte & 0x80) ? 0x7F00 : 0x0000;
            uint16_t brick_right = (bricks_byte & 0x40) ? 0x007F : 0x0000;
            bricks_byte <<= 2;

            uint16_t brick_r = (brick_color & COLOR_RED) ? brick_left | brick_right : 0;
            uint16_t brick_g = (brick_color & COLOR_GREEN) ? brick_left | brick_right : 0;
            uint16_t brick_b = (brick_color & COLOR_BLUE) ? brick_left | brick_right : 0;

            red_pix_data[i+1] = brick_r;

            green_pix_data[i] |= brick_g >> 12;
            green_pix_data[i+1] = brick_g << 4;

            blue_pix_data[i] |= brick_b >> 8;
            blue_pix_data[i+1] = brick_b << 8;
        }

        // Right wall
        red_pix_data[i+1] = 0x0F00;
        green_pix_data[i+1] = 0xF000;
        blue_pix_data[i] |= 0x000F;

        uint32_t bat = 0xFFE00000;

        // Bat
        if (line >= 0x200 && line < 0x208) {

            int bat_align_green = (bat_x >> 1) + 13 - 4;
            green_pix_data[bat_align_green >> 4] |= (bat >> (bat_align_green & 0x0F)) >> 16;
            green_pix_data[(bat_align_green >> 4) + 1] |= (bat >> (bat_align_green & 0x0F)) & 0xFFFF;

            int bat_align_blue = (bat_x >> 1) + 13 - 8;
            blue_pix_data[bat_align_blue >> 4] |= (bat >> (bat_align_blue & 0x0F)) >> 16;
            blue_pix_data[(bat_align_blue >> 4) + 1] |= (bat >> (bat_align_blue & 0x0F)) & 0xFFFF;

            int bat_align_red = (bat_x >> 1) + 13;
            red_pix_data[bat_align_red >> 4] |= (bat >> (bat_align_red & 0x0F)) >> 16;
            red_pix_data[(bat_align_red >> 4) + 1] |= (bat >> (bat_align_red & 0x0F)) & 0xFFFF;
        }

        // Ball
        if (line >= ball_y - 4 && line < ball_y + 4) {
            int ball_align_red = (ball_x >> 1) + 13;
            red_pix_data[ball_align_red >> 4] |= 0x8000 >> (ball_align_red & 0x0F);

            int ball_align_green = (ball_x >> 1) + 13 - 4;
            green_pix_data[ball_align_green >> 4] |= 0x8000 >> (ball_align_green & 0x0F);

            int ball_align_blue = (ball_x >> 1) + 13 - 8;
            blue_pix_data[ball_align_blue >> 4] |= 0x8000 >> (ball_align_blue & 0x0F);
        }

        __asm__("    dsb\n");

        uDMAChannelTransferSet(RED_DMA_CHAN | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                               red_pix_data, (void *)(SSI0_BASE + SSI_O_DR), 6);

        uDMAChannelTransferSet(GREEN_DMA_CHAN | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                               green_pix_data, (void *)(SSI1_BASE + SSI_O_DR), 6);

        uDMAChannelTransferSet(BLUE_DMA_CHAN | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                               blue_pix_data, (void *)(SSI2_BASE + SSI_O_DR), 6);

        uDMAChannelEnable(RED_DMA_CHAN);
        uDMAChannelEnable(GREEN_DMA_CHAN);
        uDMAChannelEnable(BLUE_DMA_CHAN);
    }

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    __asm__("    dsb\n");
}


void vblank_isr(void)
{
    TimerIntClear(WTIMER0_BASE, TIMER_B);
    line = -28;
}

void dma_isr(void)
{
    uDMAIntClear(1 << 19);
}

void init(void)
{
    // Set CPU clock to 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    // Set PWM clock to 80 MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Enable GPIO Ports ABCDF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Enable Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    // Enable Wide Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0));

    // Enable Serial Interface {0,1,2}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2));

    // Enable Timers 1+2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2));

    // Enable DMA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA));

    IntMasterEnable();

    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    /**********************************\
     *             HSYNC              *
    \**********************************/
    // Set PB6 as PWM output pin
    GPIOPinConfigure(GPIO_PB6_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

    // Configure Timer0A as 16-bit PWM (HSYNC)
    // Configure Timer0B as perodic (HSYNC Interrupt)
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PERIODIC);

    // 1056 px clks per line
    TimerLoadSet(TIMER0_BASE, TIMER_A, 1056*2 - 1);
    TimerLoadSet(TIMER0_BASE, TIMER_B, 1056*2 - 1);

    // Sync pulse lasts 128 px clks
    TimerMatchSet(TIMER0_BASE, TIMER_A, (1056 - 128) * 2 - 1);

    // Trigger interrupt at end of visible display area (start of front porch)
    // For now, just pick a point in the middle that allows enough time for 
    // us to race the beam
    TimerMatchSet(TIMER0_BASE, TIMER_B, 900);

    // Sync is active low
    TimerControlLevel(TIMER0_BASE, TIMER_A, true);

    // Enable interrupts
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_MATCH);
    TimerIntRegister(TIMER0_BASE, TIMER_B, hblank_isr);


    /**********************************\
     *             VSYNC              *
    \**********************************/
    // Set PC4 as PWM output pin
    GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);

    // Configure WideTimer0 as 32-bit PWM
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PERIODIC);

    // 1056 px clks per line x 628 lines per frame
    TimerLoadSet(WTIMER0_BASE, TIMER_A, 1056*2 * 628 - 1);
    TimerLoadSet(WTIMER0_BASE, TIMER_B, 1056*2 * 628 - 1);

    // Sync pulse lasts 4 lines of 1056 px clks each
    TimerMatchSet(WTIMER0_BASE, TIMER_A, 1056*2 * (628 - 4) - 1);

    // Visible area ends after sync + back porch + visible
    // (i.e. beginning of front porch)
    // TimerMatchSet(WTIMER0_BASE, TIMER_B, 1056*2 * (628 - (4 + 23 + 600)) - 1);

    // Sync is active low
    TimerControlLevel(WTIMER0_BASE, TIMER_A, true);

    // Enable Vsync interrupts
    IntEnable(INT_WTIMER0B);
    TimerIntRegister(WTIMER0_BASE, TIMER_B, vblank_isr);
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMB_MATCH | TIMER_TIMB_TIMEOUT);

    /**********************************\
     *             Color              *
    \**********************************/

    // Disable SSI while configuring
    SSIDisable(SSI0_BASE);
    SSIDisable(SSI1_BASE);
    SSIDisable(SSI2_BASE);

    // Clock based on 80 MHz main clock
    SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
    SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_SYSTEM);
    SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);

    // Set PA5 as TX (RED pixel data out)
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    // Set PD3 as TX (GREEN pixel data out)
    GPIOPinConfigure(GPIO_PD3_SSI1TX);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    // Set PB7 as TX (BLUE pixel data out)
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    // Configure SSI{1,2,3} to be a 16-bit 5 MHz master
    // Use TI's serial mode (for back-to-back transfers)
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_TI,
                       SSI_MODE_MASTER, 5000000, 16);
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_TI,
                       SSI_MODE_MASTER, 5000000, 16);
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_TI,
                       SSI_MODE_MASTER, 5000000, 16);

    // Enable SSI now that we are done configuring
    SSIEnable(SSI0_BASE);
    SSIEnable(SSI1_BASE);
    SSIEnable(SSI2_BASE);

    /**********************************\
     *          Color Timers          *
    \**********************************/

    // Configure Timers 1A, 1B, 2A as 16-bit PWM (color)
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

    // 1056 px clks per line
    TimerLoadSet(TIMER1_BASE, TIMER_A, 1056*2 - 1);
    TimerLoadSet(TIMER1_BASE, TIMER_B, 1056*2 - 1);
    TimerLoadSet(TIMER2_BASE, TIMER_A, 1056*2 - 1);

    // Start RED at: +0
    TimerMatchSet(TIMER1_BASE, TIMER_A, (1056 - (128 + 88)) * 2 - 1 - (0));

    // Start GREEN at: +16
    TimerMatchSet(TIMER1_BASE, TIMER_B, (1056 - (128 + 88)) * 2 - 1 - (64));

    // Start BLUE at: +32
    TimerMatchSet(TIMER2_BASE, TIMER_A, (1056 - (128 + 88)) * 2 - 1 - (128));

    TimerDMAEventSet(TIMER1_BASE, TIMER_DMA_CAPMATCH_A | TIMER_DMA_CAPMATCH_B);
    TimerDMAEventSet(TIMER2_BASE, TIMER_DMA_CAPMATCH_A);

    TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
    TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_MATCH | TIMER_TIMB_MATCH);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_MATCH);


    /**********************************\
     *            microDMA            *
    \**********************************/

    // Enable the uDMA controller
    uDMAEnable();

    // Set the base for the channel control table
    uDMAControlBaseSet(&dma_ctl_tbl[0]);

    // Setup channels for timer triggered DMA
    uDMAChannelAssign(UDMA_CH20_TIMER1A);
    uDMAChannelAssign(UDMA_CH21_TIMER1B);
    uDMAChannelAssign(UDMA_CH14_TIMER2A);

    // Clear any attributes
    uDMAChannelAttributeDisable(RED_DMA_CHAN, UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    uDMAChannelAttributeDisable(GREEN_DMA_CHAN, UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    uDMAChannelAttributeDisable(BLUE_DMA_CHAN, UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    uDMAChannelControlSet(RED_DMA_CHAN | UDMA_PRI_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_8);

    uDMAChannelControlSet(GREEN_DMA_CHAN | UDMA_PRI_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_8);

    uDMAChannelControlSet(BLUE_DMA_CHAN | UDMA_PRI_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_8);


    // IntEnable(INT_UDMA);
    // uDMAIntRegister(19, dma_isr);
    // uDMAChannelTransferSet(19 | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
    //                        pix_data, (void *)(SSI0_BASE + SSI_O_DR), 4);
    // uDMAChannelDisable(19);

    /**********************************\
     *             Start!             *
    \**********************************/

    // GO!
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    TimerEnable(WTIMER0_BASE, TIMER_BOTH);

    TimerEnable(TIMER1_BASE, TIMER_BOTH);
    TimerEnable(TIMER2_BASE, TIMER_A);

    // Sync up
    TimerSynchronize(TIMER0_BASE, 0);
    TimerSynchronize(TIMER0_BASE, TIMER_0A_SYNC | TIMER_0B_SYNC | WTIMER_0A_SYNC | TIMER_1A_SYNC | TIMER_1B_SYNC | TIMER_2A_SYNC);
}

void reset()
{
    int i;
    for (i = 0; i < 12; i++) {
        bricks[i] = 0xFF;
    }

    ball_x = 70;
    ball_y = 507;

    bat_x = 60;

    ball_x_vel = 0;
    ball_y_vel = 0;
}

int main(void)
{
    init();

    while (1) { 
        // Wait for VBLANK
        while(!vblank) CPUwfi();
        vblank = false;

        ball_y += ball_y_vel;
        if (ball_y < 0) {
            ball_y_vel *= -1;
        }

        if (ball_y > 600) {
            reset();
            continue;
        }

        ball_x += ball_x_vel;
        if (ball_x < 0 || ball_x > 142) {
            ball_x_vel *= -1;
        }

        int btns = ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4) & (GPIO_PIN_0 | GPIO_PIN_4);

        if (btns == GPIO_PIN_4) {
            bat_x--;
            if (bat_x < 0) {
                bat_x = 0;
            }

            if (ball_x_vel == 0 || ball_y_vel == 0) {
                ball_x_vel = -1;
                ball_y_vel = 2;
            }
        }

        if (btns == GPIO_PIN_0) {
            bat_x++;
            if (bat_x > 142 - 22) {
                bat_x = 122;
            }

            if (ball_x_vel == 0 || ball_y_vel == 0) {
                ball_x_vel = 1;
                ball_y_vel = 2;
            }
        }

        if ((ball_y < 0x204 && ball_y > 0x200 - 4) && (ball_x >= bat_x-1 && ball_x < bat_x + 22)) {
            ball_y_vel *= -1;
            if ((ball_x_vel < 0 && ball_x >= bat_x + 18) || (ball_x_vel > 0 && ball_x < bat_x + 4)) {
                ball_x_vel *= -1;
            }
        }

        int ball_col = ((ball_x >> 1) - 3) >> 3;
        if (ball_col >= 0 && ball_col < 8 && ((ball_x & 0x0F) >= 2)) {

            int top_line = ball_y - 4;
            uint8_t top_bricks = (top_line >= 0x40 && top_line < 0x100) ? bricks[(top_line - 0x40) >> 4] : 0x00;
            
            int bot_line = ball_y + 4;
            uint8_t bot_bricks = (bot_line >= 0x40 && bot_line < 0x100) ? bricks[(bot_line - 0x40) >> 4] : 0x00;

            if (top_bricks & (0x80 >> ball_col) && bot_bricks & (0x80 >> ball_col)) {
                bricks[(top_line - 0x40) >> 4] &= ~(0x80 >> ball_col);
                bricks[(bot_line - 0x40) >> 4] &= ~(0x80 >> ball_col);
                if (((ball_x & 0x0F) == 2 && ball_x_vel > 0) || ((ball_x & 0x0F) == 0xF && ball_x_vel < 0)) {
                    ball_x_vel *= -1;
                    ball_x += ball_x_vel;
                } else {
                    // Plow through? Or bounce?
                    ball_x_vel *= -1;
                    ball_x += ball_x_vel;
                }
            } else if (top_bricks & (0x80 >> ball_col)) {
                bricks[(top_line - 0x40) >> 4] &= ~(0x80 >> ball_col);
                if (((ball_x & 0x0F) == 2 && ball_x_vel > 0) || ((ball_x & 0x0F) == 0xF && ball_x_vel < 0)) {
                    ball_x_vel *= -1;
                    ball_x += ball_x_vel;
                } else {
                    ball_y_vel *= -1;
                    ball_y += ball_y_vel;
                }
            } else if (bot_bricks & (0x80 >> ball_col)) {
                bricks[(bot_line - 0x40) >> 4] &= ~(0x80 >> ball_col);
                if (((ball_x & 0x0F) == 2 && ball_x_vel > 0) || ((ball_x & 0x0F) == 0xF && ball_x_vel < 0)) {
                    ball_x_vel *= -1;
                    ball_x += ball_x_vel;
                } else {
                    ball_y_vel *= -1;
                    ball_y += ball_y_vel;
                }
            }
            
        }

    }
}
