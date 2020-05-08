// RTOS Framework - Spring 2020
// J Losh

// Student Name:Jeffrin Stephen
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "ourstring.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED(PF2)
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED(PA2)
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board green LED(PA3)
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED(PA4)
#define GREEN_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board orange LED(PE0)
#define PUSH_BUTTON0   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) //on-board pin(PC4)
#define PUSH_BUTTON1   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) //on-board pin(PC5)
#define PUSH_BUTTON2   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) //on-board pin(PC6)
#define PUSH_BUTTON3   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) //on-board pin(PC7)
#define PUSH_BUTTON4   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4))) //on-board pin(PD6)
#define PUSH_BUTTON5   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4))) //on-board pin(PD7)

#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define GREEN_LED_MASK 1
#define YELLOW_LED_MASK 16
#define ORANGE_LED_MASK 8

#define PUSH_BUTTON0_MASK 16
#define PUSH_BUTTON1_MASK 32
#define PUSH_BUTTON2_MASK 64
#define PUSH_BUTTON3_MASK 128
#define PUSH_BUTTON4_MASK 64
#define PUSH_BUTTON5_MASK 128
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
extern uint32_t push();
extern uint32_t pop();
extern uint32_t setPSP(uint32_t *SP);
extern uint32_t setStack_pointer(uint32_t *SP);
extern uint32_t* setStack_pointer2();
extern uint32_t* setStack_pointer3();
extern uint32_t testValues1_R11_to_R4();
extern uint32_t testValues2_R4_to_R11();
extern uint32_t get_R0();
// function pointer
typedef void (*_fn)();
bool PRIORITY=true;
bool PRIORITY_INHERITANCE=false;

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    char name[16];
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint8_t priority=0;
uint8_t prCount=0;
uint8_t VAR=0;
uint32_t stack[10][512];
bool preemption=true;
uint32_t SUMS[MAX_TASKS];
uint8_t FLAG=0;
uint16_t systick_count=1000;
uint32_t total=0;
uint32_t Usage_by_task[MAX_TASKS];
char stren[11];
// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{ __asm(" SVC #150 ");
}

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

// Initialize UART0
void initUart0()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)               // wait if uart0 tx fifo full
    yield();
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)               // wait if uart0 rx fifo empty
    yield();
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    uint32_t SUM[2];
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];


//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }

}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    uint8_t i;
    if(!PRIORITY)
    {
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    }
    else
    {while(!ok)
     {for(i=1;i<=MAX_TASKS;i++)
    {VAR=(i+prCount-1)%MAX_TASKS;
     ok=((tcb[VAR].priority==priority)&&(tcb[VAR].state == STATE_READY || tcb[VAR].state == STATE_UNRUN));
             if(ok)
             {  task=VAR;
                 priority=0;
                 prCount=VAR+1;
             return task;
             }
             }
     priority++;
     }
    }
    return task;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    //strCopy2(name,tcb[i].name);
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            strCopy2(name,tcb[i].name);
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][512];
            tcb[i].spInit=tcb[i].sp;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{__asm(" SVC #155 ");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{__asm(" SVC #154 ");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{__asm(" SVC #156 ");
}

void get_timeDurations()
{__asm(" SVC #157 ");
}

struct semaphore* createSemaphore(uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
    }
    return pSemaphore;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    _fn fn;
taskCurrent = rtosScheduler();
NVIC_ST_CTRL_R=0;
NVIC_ST_CURRENT_R=0;
NVIC_ST_RELOAD_R=39999;
NVIC_ST_CTRL_R=4|2|1;
setStack_pointer(tcb[taskCurrent].sp);
setPSP(tcb[taskCurrent].sp);
fn=(_fn)tcb[taskCurrent].pid;
(*fn)();
}

void push_xpsr_to_R0()
{
    uint32_t* x=setStack_pointer2();
    x--;
    *x=0x01000000;
     x--;
     *x=tcb[taskCurrent].pid;
     x--;
     x--;
     x--;
     x--;
     x--;
     x--;
     setPSP(x);
    tcb[taskCurrent].state = STATE_READY;
}

uint8_t get_svc_number()
{
uint32_t* y=setStack_pointer2();
y++;
y++;
y++;
y++;
y++;
y++;
uint32_t z=*y;
z=z-2;
uint32_t *c= z;
uint8_t s=*c;
return s;
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{__asm(" SVC #151 ");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{__asm(" SVC #152 ");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{__asm(" SVC #153 ");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{uint8_t i;
systick_count--;
for(i=0;i<MAX_TASKS;i++)
{
if(tcb[i].state==STATE_DELAYED)
{
    if(tcb[i].ticks>0)
    tcb[i].ticks--;
    else
    tcb[i].state =STATE_READY;
}
}
if(preemption)
    NVIC_INT_CTRL_R=0x10000000;
if(systick_count==0)
{systick_count=1000;
tcb[taskCurrent].SUM[1-FLAG]=0;
 FLAG^=1;
}

}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
   // RED_LED=1;
    uint8_t N,i,x,ttKill_or_Restore;
    uint32_t*z;
    _fn fn;
    struct semaphore *s;
    N=get_svc_number();
    switch(N)
    {
    case 150:NVIC_INT_CTRL_R=0x10000000;
               break;
    case 151:tcb[taskCurrent].state=STATE_DELAYED;
               tcb[taskCurrent].ticks=*setStack_pointer2();
               NVIC_INT_CTRL_R=0x10000000;
               break;
    case 152: s= (struct semaphore*)*setStack_pointer2();
             if(s->count > 0)
             { s->count--;
             if(PRIORITY_INHERITANCE)
                 tcb[taskCurrent].semaphore=s;
             }
             else
             {
              s->processQueue[s->queueSize]=taskCurrent;
              tcb[taskCurrent].state=STATE_BLOCKED;
              tcb[taskCurrent].semaphore=s;
              s->queueSize++;
              if(PRIORITY_INHERITANCE)
              {for(x=0;x<MAX_TASKS;x++)
              {if((taskCurrent!=x)&&(tcb[x].semaphore==s))
              {tcb[taskCurrent].priority=tcb[x].priority;
                break;
              }
              }
              }
              else
                  tcb[taskCurrent].priority=tcb[taskCurrent].currentPriority;
              NVIC_INT_CTRL_R=0x10000000;
             }
             break;
   case 153: s= (struct semaphore*)*setStack_pointer2();
             s->count++;
             if(s->processQueue[0]!='\0')
             {
              tcb[s->processQueue[0]].state=STATE_READY;
              for(i=s->count;i<(s->queueSize);i++)
                  s->processQueue[i]=s->processQueue[i+1];
              s->count--;
              s->queueSize--;
              if(PRIORITY_INHERITANCE)
                  tcb[taskCurrent].priority=tcb[taskCurrent].currentPriority;
             }
             break;
   case 154: fn=*setStack_pointer2();
             for(x=0;x<MAX_TASKS;x++)
             {if(tcb[x].pid==fn)
             {  ttKill_or_Restore=x;
             break;
             }
             }

             s=tcb[ttKill_or_Restore].semaphore;
             if(ttKill_or_Restore==2)
                 tcb[ttKill_or_Restore].state=STATE_INVALID;
             else{
            if(s->processQueue[0]!='\0')
             {
              tcb[s->processQueue[0]].state=STATE_INVALID;
              for(i=s->count;i<(s->queueSize);i++)
                  s->processQueue[i]=s->processQueue[i+1];
              s->queueSize--;
             }
             }
             break;
   case 155: fn=*setStack_pointer2();
             for(x=0;x<MAX_TASKS;x++)
             {if(tcb[x].pid==fn)
             {  ttKill_or_Restore=x;
             break;
             }
             }
             if(tcb[ttKill_or_Restore].state==STATE_INVALID)
             {tcb[i].sp = tcb[i].spInit;
             tcb[ttKill_or_Restore].state=STATE_UNRUN;
             }
             break;
   case 156: fn=*setStack_pointer2();
             z=setStack_pointer2();
             z++;
             i=*z;
             for(x=0;x<MAX_TASKS;x++)
             {if(tcb[x].pid==fn)
             {  ttKill_or_Restore=x;
             break;
             }
             }
             tcb[ttKill_or_Restore].priority=i;
             break;
   case 157: for(x=0;x<MAX_TASKS-1;x++)
             {SUMS[x]=tcb[x].SUM[1-FLAG];
              }
             total=0;
             for(x=0;x<MAX_TASKS-1;x++)
             {total+=SUMS[x];
             }
             break;
    default:   NVIC_INT_CTRL_R=0x10000000;
               break;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
push();
testValues2_R4_to_R11();
tcb[taskCurrent].sp=setStack_pointer2();
TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer
tcb[taskCurrent].SUM[1-FLAG]=TIMER1_TAV_R;
TIMER1_TAV_R=0;
taskCurrent = rtosScheduler();
TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
if(tcb[taskCurrent].state == STATE_READY)
{
setPSP(tcb[taskCurrent].sp);
pop();
}
else
{
setPSP(tcb[taskCurrent].sp);
push_xpsr_to_R0();
}
}


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    // Enable GPIO port F, A, E, C and D peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD;

    // Configure LED and pushbutton pins

    GPIO_PORTD_LOCK_R=0x4C4F434B;
    GPIO_PORTD_CR_R = PUSH_BUTTON5_MASK;

    GPIO_PORTF_DIR_R = BLUE_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = BLUE_LED_MASK;  // enable LEDs and pushbuttons

    GPIO_PORTA_DIR_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;  // enable LEDs and pushbuttons

    GPIO_PORTE_DIR_R = GREEN_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTE_DR2R_R = GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = GREEN_LED_MASK;  // enable LEDs and pushbuttons

    GPIO_PORTC_DEN_R = PUSH_BUTTON0_MASK | PUSH_BUTTON1_MASK | PUSH_BUTTON2_MASK | PUSH_BUTTON3_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTC_PUR_R = PUSH_BUTTON0_MASK | PUSH_BUTTON1_MASK | PUSH_BUTTON2_MASK | PUSH_BUTTON3_MASK; // enable internal pull-up for push button

    GPIO_PORTD_DEN_R = PUSH_BUTTON4_MASK | PUSH_BUTTON5_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTD_PUR_R = PUSH_BUTTON4_MASK | PUSH_BUTTON5_MASK;

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD|TIMER_TAMR_TACDIR;          // configure for (count UP)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}



// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{   if(PUSH_BUTTON0==0)
    return 1;
    else if(PUSH_BUTTON1==0)
    return 2;
    else if(PUSH_BUTTON2==0)
    return 4;
    else if(PUSH_BUTTON3==0)
    return 8;
    else if(PUSH_BUTTON4==0)
    return 16;
    else if(PUSH_BUTTON5==0)
    return 32;
    else
    return 0;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        testValues1_R11_to_R4();
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 32)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
//           your solution should not use C library calls for strings, as the stack will be too large
uint8_t count=0;
char c;
#define MAXCHAR    80
#define MAX_CHARS    80
#define MAX_FIELDS 15
char str[MAX_CHARS+1];
uint8_t pos[MAX_FIELDS];
uint8_t argCount;
uint8_t x;
uint8_t tmp;
//Step 2
void getString(char str[], uint8_t maxchar)
{ while(count<maxchar)
  {

   c=getcUart0();

   if(c==8||c==127)
   {
       if(count>0)
    {
       count-- ;
    }
       else
       continue;
   }


   else if(c==10||c==13)
   {
       str[count]=0;
       break;
   }

   else if(c>=32)
   {
       str[count++]=c;
   }

   if(count==MAXCHAR)
   {str[count]=0;
      break;
     }
   else
       continue;
  }
  count=0;
    }

//Step3
void parseString(char str[], uint8_t pos[], uint8_t maxfields, uint8_t *pArgCount)
{   uint8_t i=0;
    uint8_t j=0;

     *pArgCount=0;
    while(i<maxfields)
    {
     c=str[i];
    if(((str[i+1]>=48 && str[i+1]<=57) || (str[i+1]>=65 && str[i+1]<=90) || (str[i+1]>=97 && str[i+1]<=122) || (str[i+1]==46) || (str[i+1]==45)|| (str[i+1]==38))&&(!((c>=48 && c<=57) || (c>=65 && c<=90) || (c>=97 && c<=122) || (c==46) || (c==45) || (c==38))))
              {
                  pos[j]=i+1;
                  *pArgCount=j+1;
                  j++;
              }
    else if((i==0)&&((str[i]>=48 && str[i]<=57) || (str[i]>=65 && str[i]<=90) || (str[i]>=97 && str[i]<=122) || (str[i]==46) || (str[i]==45) || (str[i]==38)))
    {
        pos[j]=i;
        *pArgCount=j+1;
        j++;

    }
     i++;
    }
    i=0;
    while(i<maxfields)
    {

        if(!((str[i]>=48 && str[i]<=57) || (str[i]>=65 && str[i]<=90) || (str[i]>=97 && str[i]<=122) || (str[i]==46) || (str[i]==45) || (str[i]==38)))
         str[i]='\0';
        i++;

    }


}

//Step4
char *getArgString(uint8_t argNo)
{
  if(argNo<argCount)
      return &str[pos[argNo]];
  else
      return '\0';
}

uint32_t getValueArgInt(uint8_t argNo)
{
  uint32_t value;
  value = ascii_to_integer(getArgString(argNo));
  return value;
}

//Step5
bool isCommand(char strCMD[], uint8_t minArg)
{   x = strCompare(strCMD, getArgString(0));
    if((!x) && (minArg<argCount))
    {

        return true;
    }
    else
        return false;
}

void shell()
{
    while (true)
    {
        //sTEP2
             _fn fn;
             uint32_t s=0;
             uint8_t tmp;
             putsUart0("\r\n");
             putsUart0("Enter Command:");
             getString(str, MAXCHAR);
             //sTEP3 & sTEP4
             parseString(str, pos, MAX_FIELDS, &argCount);
             if(isCommand("reboot",0))
                  { NVIC_APINT_R = 0x05FA0004;
                  }

             if(isCommand("pidof",1))
             {
                 if(!strCompare("Idle",getArgString(1)))
                 {s=tcb[0].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("LengthyFn",getArgString(1)))
                 {s=tcb[1].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("Flash4Hz",getArgString(1)))
                 {s=tcb[2].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("OneShot",getArgString(1)))
                 {s=tcb[3].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("Important",getArgString(1)))
                 {s=tcb[6].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("Shell",getArgString(1)))
                 {s=tcb[8].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("ReadKeys",getArgString(1)))
                 {s=tcb[4].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("Debounce",getArgString(1)))
                 {s=tcb[5].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else if(!strCompare("Uncoop",getArgString(1)))
                 {s=tcb[7].pid;
                 putsUart0("PID NO: ");
                 my_sprintf(stren,s);
                 putsUart0(stren);
                 }
                 else
                     putsUart0("The entered process is not available in the RTOS");

             }
             if(isCommand("ipcs",0))
             {for(s=0;s<4;s++)
             {  if(s==0)
                 strCopy1("keyPressed",semaphores[s].name);
             else if(s==1)
                 strCopy1("keyReleased",semaphores[s].name);
             else if(s==2)
                 strCopy1("flashReq",semaphores[s].name);
             else
                 strCopy1("resource",semaphores[s].name);

             }
             putsUart0("Semaphore          Semaphore count              queuesize");
             putsUart0("\r\n");
             putsUart0("---------          ---------------              ---------");
             for(s=0;s<4;s++)
             {putsUart0("\r\n");
             my_sprintf(stren,semaphores[s].count);
             putsUart0(semaphores[s].name);
             if(s==0)
             putsUart0("                  ");
             else if(s==1)
             putsUart0("                 ");
             else if(s==2)
                 putsUart0("                    ");
             else
                 putsUart0("                    ");
             putsUart0(stren);
             my_sprintf(stren,semaphores[s].queueSize);
             putsUart0("                       ");
             putsUart0(stren);
             }
             }
             if(isCommand("priority",1))
             {if(!strCompare("on",getArgString(1)))
                 PRIORITY=true;
             else if(!strCompare("off",getArgString(1)))
                 PRIORITY=false;
             else
             {putsUart0("\r\n");
             putsUart0("Invalid Command");
             }
             }

             if(isCommand("Kill",1))
             {fn=getValueArgInt(1);
             destroyThread(fn);
             }

             if(isCommand("PI",1))
             {if(!strCompare("on",getArgString(1)))
                 PRIORITY_INHERITANCE=true;
             else if(!strCompare("off",getArgString(1)))
                 PRIORITY_INHERITANCE=false;
             else
             {putsUart0("\r\n");
             putsUart0("Invalid Command");
             }
             }

             if(isCommand("Preemption",1))
             {if(!strCompare("on",getArgString(1)))
                 preemption=true;
             else if(!strCompare("off",getArgString(1)))
                 preemption=false;
             else
             {putsUart0("\r\n");
             putsUart0("Invalid Command");
             }
             }

             if(isCommand("Restore",1))
             {fn=getValueArgInt(1);
             restartThread(fn);
             }

             if(isCommand("Idle",1) || isCommand("LengthyFn",1) || isCommand("Flash4Hz",1) || isCommand("OneShot",1) || isCommand("ReadKeys",1) || isCommand("Debounce",1) || isCommand("Important",1) || isCommand("Uncoop",1) || isCommand("Shell",1))
             {if(!strCompare("&",getArgString(1)))
             {for(x=0;x<MAX_TASKS-1;x++)
              {strCopy2(getArgString(0),stren);
              if(!strCompare(stren,tcb[x].name))
                  break;
              }
             fn=tcb[x].pid;
             restartThread(fn);
             }
             else
             {putsUart0("\r\n");
             putsUart0("Invalid Command");
             }
             }

             if(isCommand("ps",0))
             {get_timeDurations();
             putsUart0("PID NUMBER          PROCESS        CURRENT PROCESS STATE          CPU TIME %");
                          putsUart0("\r\n");
                          putsUart0("----------          -------        ---------------------          ----------");
                          putsUart0("\r\n");
                          for(x=0;x<MAX_TASKS-1;x++)
                          {
                            Usage_by_task[x]=(SUMS[x]*10000)/total;
                            s=tcb[x].pid;
                            my_sprintf(stren,s);
                            putsUart0(stren);
                            putsUart0("                ");
                            putsUart0(tcb[x].name);
                            if(x==0)
                            putsUart0("                    ");
                            else if(x==1)
                            putsUart0("               ");
                            else if(x==2)
                            putsUart0("                ");
                            else if(x==3)
                            putsUart0("                 ");
                            else if(x==4)
                            putsUart0("                ");
                            else if(x==5)
                            putsUart0("                ");
                            else if(x==6)
                            putsUart0("               ");
                            else if(x==7)
                            putsUart0("                  ");
                            else
                                putsUart0("                   ");
                            if(tcb[x].state==0)
                                putsUart0("INVALID");
                            else if(tcb[x].state==1)
                                putsUart0("UNRUN");
                            else if(tcb[x].state==2)
                                putsUart0("READY");
                            else if(tcb[x].state==3)
                                putsUart0("DELAYED");
                            else
                                putsUart0("BLOCKED");
                            if(PRIORITY_INHERITANCE)
                            {
                            if(x==0)
                            putsUart0("                  ");
                            else if(x==1)
                            putsUart0("                ");
                            else if(x==2)
                                putsUart0("                ");
                            else if(x==3)
                                putsUart0("                ");
                            else if(x==4)
                                putsUart0("                  ");
                            else if(x==5)
                                putsUart0("                ");
                            else if(x==6)
                                putsUart0("                ");
                            else if(x==7)
                                putsUart0("                  ");
                            else
                                putsUart0("                  ");
                            }
                            else
                            {
                                if(x==0)
                                putsUart0("                  ");
                                else if(x==1)
                                putsUart0("                  ");
                                else if(x==2)
                                    putsUart0("                ");
                                else if(x==3)
                                    putsUart0("                ");
                                else if(x==4)
                                    putsUart0("                  ");
                                else if(x==5)
                                    putsUart0("                ");
                                else if(x==6)
                                    putsUart0("                ");
                                else if(x==7)
                                    putsUart0("                  ");
                                else
                                    putsUart0("                  ");

                            }

                            tmp=Usage_by_task[x]%100;
                            s=Usage_by_task[x]/100;
                            my_sprintf(stren,s);
                            putsUart0(stren);
                            putsUart0(".");
                            my_sprintf(stren,tmp);
                            putsUart0(stren);
                            putsUart0("\r\n");
                            }
             }
             if(!(isCommand("reboot",0) || isCommand("pidof",1) || isCommand("ipcs",0) ||isCommand("Kill",1) || isCommand("priority",1) || isCommand("Preemption",1) || isCommand("PI",1) || isCommand("Restore",1) || isCommand("Idle",1) || isCommand("LengthyFn",1) || isCommand("Flash4Hz",1) || isCommand("OneShot",1) || isCommand("ReadKeys",1) || isCommand("Debounce",1) || isCommand("Important",1) || isCommand("Uncoop",1) || isCommand("Shell",1) || isCommand("ps",0)))
             {putsUart0("Invalid Command");
             putsUart0("\r\n");
             }
             putsUart0("\r\n");
             yield();


    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initUart0();
    initRtos();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);
    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 12, 1024);
    ok &= createThread(shell, "Shell", 12, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
