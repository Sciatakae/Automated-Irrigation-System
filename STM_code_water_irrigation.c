  /*
 * app.c
 *
 * Starter application for the STM32 automatic watering project.
 * This follows the same App_Init / App_MainLoop style used in previous labs.
 */

#include "stdio.h"
#include "string.h"

#include "app.h"

// defining the parameters of the entire project, think of this as using the code to represent the real parts of the physical project
// the code also defines the samples that are being converted the soil sensor (remembers its jsut an ADC conversion)

// defining some sort of auto stop to make sure the pump and relay dont just run water in some error or bug 
// also defining the threshhold of what dry soil is according to you. so mess around with it later and see if you can get good readings


/*
 * Relay polarity:
 * Many relay boards are active-low, which means driving the pin LOW turns
 * the relay on. If your relay behaves the opposite way, swap these two.
 */
#define RELAY_ACTIVE_STATE     GPIO_PIN_RESET
#define RELAY_INACTIVE_STATE   GPIO_PIN_SET

/* GPIO used for the relay control signal. */
#define PUMP_PORT              GPIOA
#define PUMP_PIN               GPIO_PIN_1

/* Onboard LED used as a simple heartbeat indicator. */
#define LED_PORT               GPIOA
#define LED_PIN                GPIO_PIN_5

/* ADC averaging helps smooth out noisy soil sensor readings. */
#define ADC_AVG_SAMPLES        10U

/* If moisture percent drops below this, auto mode starts watering. */
#define DRY_THRESHOLD_PERCENT  30U

/* Safety limits so the pump cannot run forever. */
#define MAX_PUMP_SECONDS       10U
#define COOLDOWN_DURATION      30U

// still need to really calibrate, the decent moist reading never went above 2700, so maybe the default 3000 would be the best approach? 

// gotta mess with this again later
/*
 * Replace these values after calibration.
 * Capacitive sensors usually read higher when dry and lower when wet,
 * so we map raw ADC values into a moisture percentage using this range.
 */
#define SENSOR_DRY_RAW         2700U
#define SENSOR_WET_RAW         1500U

// ADC, TIMERS, AND UART handles 

/* These handles are created by STM32Cube-generated code in main.c. */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;


// ?? 

/* Internal helper functions used only inside this file. */
static void ShowCommands(void);
static void UART_SendString(const char *text);
static void Pump_Set(uint8_t on);
static void PrintStatus(void);
static void ProcessCommand(void);
static uint32_t ReadMoistureRawAverage(void);
static uint32_t RawToPercent(uint32_t raw);
static uint8_t CommandMatches(const char *command, const char *alias1, const char *alias2);


// ihh have to study this section a little more because im not sure what the benefit is here

// volatile : type qualifier that informs the compiler that a variable's value can be changed by something outside the program's control at any time
// volatile is absolutely essential for a project involving ADC and UART because both rely on hardware registers and asynchronous events that the compiler cannot track


/*
 * These variables are shared between normal code and interrupt callbacks,
 * so they are marked volatile. That tells the compiler their values may
 * change unexpectedly outside the current flow of execution.
 */
volatile uint8_t rxByte = 0;
volatile uint8_t rxReady = 0;
volatile uint8_t sampleRequested = 0;
volatile uint8_t pumpActive = 0;
volatile uint8_t cooldownActive = 0;

/* Start in manual mode so the pump does not automatically run on boot. */
volatile uint8_t autoMode = 0;

/* When stream mode is enabled, the board prints status every sample period. */
volatile uint8_t streamMode = 0;

/* These flags let ISRs request messages without printing inside the ISR. */
volatile uint8_t safetyMessagePending = 0;
volatile uint8_t cooldownMessagePending = 0;

/* Counters updated once per second by TIM2. */
volatile uint32_t pumpOnSeconds = 0;
volatile uint32_t cooldownSeconds = 0;

/* UART command buffer and most recent sensor results. */
static char rxBuffer[32];
static uint8_t rxIndex = 0;
static uint32_t lastRaw = 0;
static uint32_t lastPercent = 0;

// a big thing is to make sure the pump doesnt turn on and just stay on like before
// ill note here to make sure the relay gets 3.3V and not the 5V according to the data sheet.. at that point the 5v was just forcing the thing to be on at all times

void App_Init(void)
{
    /* Force the relay to the OFF state during startup. */
    Pump_Set(0);

    /* Start with the LED off. TIM2 will toggle it later. */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    /* Boot banner shown once in PuTTY. */
    UART_SendString("\r\n-----------------------------\r\n");
    UART_SendString(" STM32 Automatic Watering Terminal \r\n");
    UART_SendString("-----------------------------\r\n");
    UART_SendString("Type a command, then press Enter.\r\n");
    // add the commands to actuallu show what the command buttons are instead of guessing.
    // also maybe i should add soemthing that will remind me that the inputs dont pop up UNLESS I ACTIVATE IT ON PUTTY(and this is just for windows, ive yet to have a signle sucessful run on the macbook)
    // im thinking about making this connect to the oldmacbook as a client and that computer can just run the inside plants or something
    // this is April 19 notes,

    ShowCommands();

    /*
     * TIM2:
     * 1-second interrupt for heartbeat LED, pump safety timeout,
     * and cooldown timing.
     */
    HAL_TIM_Base_Start_IT(&htim2);

    /*
     * TIM3:
     * 1-second interrupt that requests a new moisture sample.
     * The actual ADC read is done in the main loop, not in the ISR.
     */
    HAL_TIM_Base_Start_IT(&htim3);

    /* Start UART receive interrupt so commands can come in from PuTTY. */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rxByte, 1);
}

void App_MainLoop(void)
{
    /* If a full command line arrived over UART, process it here. */
    if (rxReady != 0U) {
        rxReady = 0;
        ProcessCommand();
    }

    /*
     * TIM3 does not read the ADC directly.
     * It only sets sampleRequested = 1.
     * The main loop sees that flag and performs the sample here.
     */
    if (sampleRequested != 0U) {
        sampleRequested = 0;

        /* Read and average several ADC samples, then convert to percent. */
        lastRaw = ReadMoistureRawAverage(); // i feel like the data gathered isnt as sensitive as the ECG data, so its okay to avg out the outputs since its not precise sensitive data
        lastPercent = RawToPercent(lastRaw);

        /*
         * Auto watering decision:
         * If auto mode is enabled, the pump is currently off, and we are not
         * in cooldown, then turn the pump on when the moisture is too low.
         */
        if ((autoMode != 0U) && (pumpActive == 0U) && (cooldownActive == 0U)) {
            if (lastPercent < DRY_THRESHOLD_PERCENT) {
                Pump_Set(1);
                pumpOnSeconds = 0;
                UART_SendString("AUTO: soil is dry, pump ON\r\n");
            }
        }

        /*
         * Once the soil reading rises above the threshold plus a small margin,
         * stop the pump and begin cooldown so it does not chatter on/off.
         * 
         * the cooldown was a good idea because again it prevents the machine from going crazy and flooding everything
         */
        if ((pumpActive != 0U) && (lastPercent >= DRY_THRESHOLD_PERCENT + 5U)) {
            Pump_Set(0);
            cooldownActive = 1;
            cooldownSeconds = 0;
            UART_SendString("AUTO: soil recovered, pump OFF\r\n");
        }

        /* Only print every sample if live streaming is enabled. */
        if (streamMode != 0U) {
            PrintStatus();
        }
    }

    /* Timer ISRs set these flags; the actual UART prints happen safely here. */
    if (safetyMessagePending != 0U) {
        safetyMessagePending = 0;
        UART_SendString("SAFETY: max pump time reached\r\n");
    }

    if (cooldownMessagePending != 0U) {
        cooldownMessagePending = 0;
        UART_SendString("Cooldown finished\r\n");
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *p_htim)
{
    /*
     * TIM2 runs once per second and handles all "timekeeping" behavior.
     */
    if (p_htim->Instance == TIM2) {
        /* Blink the onboard LED so you know the firmware is alive. */
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        /* If the pump is running, count how long it has been on. */
        if (pumpActive != 0U) {
            pumpOnSeconds++;
            if (pumpOnSeconds >= MAX_PUMP_SECONDS) {
                /* Safety shutoff if the pump runs too long. */
                Pump_Set(0);
                cooldownActive = 1;
                cooldownSeconds = 0;
                safetyMessagePending = 1;
            }
        }

        /* Cooldown prevents immediately turning the pump back on. */
        if (cooldownActive != 0U) {
            cooldownSeconds++;
            if (cooldownSeconds >= COOLDOWN_DURATION) {
                cooldownActive = 0;
                cooldownSeconds = 0;
                cooldownMessagePending = 1;
            }
        }
    }

    /* TIM3 simply requests a fresh sensor sample once per second. */
    if (p_htim->Instance == TIM3) {
        sampleRequested = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *p_huart)
{
    if (p_huart->Instance == USART2) {
        /*
         * Build a command string one character at a time.
         * A command is considered complete when Enter is pressed.
         */
        if ((rxByte == '\r') || (rxByte == '\n')) {
            if (rxIndex > 0U) {
                /* Null-terminate the received string so strcmp can use it. */
                rxBuffer[rxIndex] = '\0';
                rxIndex = 0;
                rxReady = 1;
            }
        } else if (rxIndex < (sizeof(rxBuffer) - 1U)) {
            /* Store normal characters into the command buffer. */
            rxBuffer[rxIndex++] = (char)rxByte;
        } else {
            /* If the buffer overflows, clear it and start over. */
            rxIndex = 0;
        }

        /* Re-arm UART interrupt so the next character can be received. */
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rxByte, 1);
    }
}

// below are the actual commands which you have to make sure they come up in the beginnng statement of theterminal or else anyone thats connected isnt gonna know wahts going on 

// h,s,stream on, stream off, auto, manual... and yes you type the whole thing out, but i can change it to numbers or something, i just have to make sure its not something stupid that can overload the controls 
// i mean like accidentally pressing 4 buttons at once and the thing goes haweite for no reason

static void ProcessCommand(void)
{
    /* help / h */
    if (CommandMatches(rxBuffer, "help", "h") != 0U) {
        ShowCommands();
        return;
    }

    /* status / s */
    if (CommandMatches(rxBuffer, "status", "s") != 0U) {
        PrintStatus();
        return;
    }

    /* Enable continuous printing once per sample period. */
    if (strcmp(rxBuffer, "stream on") == 0) {
        streamMode = 1;
        UART_SendString("Live status stream enabled\r\n");
        return;
    }

    /* Disable continuous printing. */
    if (strcmp(rxBuffer, "stream off") == 0) {
        streamMode = 0;
        UART_SendString("Live status stream disabled\r\n");
        return;
    }

    /* auto / a */
    if (CommandMatches(rxBuffer, "auto", "a") != 0U) {
        autoMode = 1;
        UART_SendString("Auto mode enabled\r\n");
        PrintStatus();
        return;
    }

    /* manual / m */
    if (CommandMatches(rxBuffer, "manual", "m") != 0U) {
        autoMode = 0;
        Pump_Set(0);
        UART_SendString("Manual mode enabled\r\n");
        PrintStatus();
        return;
    }

    /* Manual ON command aliases. */          // here is just different variations of what can close the relay and dispense the water youll see this below too for the water off section 
    if ((strcmp(rxBuffer, "on") == 0) ||
        (strcmp(rxBuffer, "open") == 0) ||
        (strcmp(rxBuffer, "1") == 0)) {

        /* Manual commands automatically leave auto mode. */
        autoMode = 0;

        if (cooldownActive != 0U) {
            UART_SendString("Pump blocked: cooldown active\r\n");
            return;
        }

        Pump_Set(1);
        pumpOnSeconds = 0;
        UART_SendString("Pump ON\r\n");
        PrintStatus();
        return;
    }

    /* Manual OFF command aliases. */ // if you didnt see it the pump manual in options are just above 
    if ((strcmp(rxBuffer, "off") == 0) ||
        (strcmp(rxBuffer, "close") == 0) ||
        (strcmp(rxBuffer, "0") == 0)) {
        Pump_Set(0);
        UART_SendString("Pump OFF\r\n");
        PrintStatus();
        return;
    }

    UART_SendString("Unknown command. Type: help\r\n");
}

static void ShowCommands(void)
{
    /* Friendly help menu shown at startup and when "help" is entered. */

    // heres the list of everything that you can press, but you have to press help first, so make sure the help otion is there
    // also you might wanna s,ow down how many readings percond are displayed, although it doesnt matter if you can just make the menhu show up again 
    UART_SendString("Commands:\r\n");
    UART_SendString("  help or h      - show commands\r\n");
    UART_SendString("  status or s    - print moisture and pump status\r\n");
    UART_SendString("  auto or a      - enable automatic watering\r\n");
    UART_SendString("  manual or m    - disable auto mode\r\n");
    UART_SendString("  on/open/1      - turn pump on manually\r\n");
    UART_SendString("  off/close/0    - turn pump off manually\r\n");
    UART_SendString("  stream on      - print readings continuously\r\n");
    UART_SendString("  stream off     - stop continuous printing\r\n");
    UART_SendString("Press Enter after each command.\r\n");
}

static void UART_SendString(const char *text)
{
    /* Blocking UART transmit is fine here because this helper is only used
     * from normal code, not from the timer ISRs.
     */
    HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

// RELAY PIN LOGIC

static void Pump_Set(uint8_t on)
{
    /*
     * This is the only place that should directly control the relay pin.
     * That keeps the pump ON/OFF behavior centralized and easier to debug.
     */
    if (on != 0U) {
        HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, RELAY_ACTIVE_STATE);
        pumpActive = 1;
    } else {
        HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, RELAY_INACTIVE_STATE);
        pumpActive = 0;
        pumpOnSeconds = 0;
    }
}

// this is where we read the AVERAGE of the raw moisture readings and have the samples based on the average of this
// again this is because taking the average reduces the noise; this isalso not crucial data at every single point, we havve know how moist the soil is and that is better seen over time to see how the mpoisture changes

static uint32_t ReadMoistureRawAverage(void)
{
    uint32_t sum = 0;
    uint32_t i = 0;

    /* Take multiple ADC readings and average them to reduce noise. */
    for (i = 0; i < ADC_AVG_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(5);
    }

    return sum / ADC_AVG_SAMPLES;
}

static uint32_t RawToPercent(uint32_t raw)
{
    int32_t percent;
    int32_t range = (int32_t)SENSOR_DRY_RAW - (int32_t)SENSOR_WET_RAW;

    /* Protect against a bad calibration range. */
    if (range <= 0) {
        return 0U;
    }

    /* Convert raw ADC value into a 0-100 moisture percentage. */
    percent = ((int32_t)SENSOR_DRY_RAW - (int32_t)raw) * 100 / range;

    /* Clamp result in case the raw value is outside the calibration range. */
    if (percent < 0) {
        percent = 0;
    }
    if (percent > 100) {
        percent = 100;
    }

    return (uint32_t)percent;
}

static uint8_t CommandMatches(const char *command, const char *alias1, const char *alias2)
{
    /* Small helper so commands can have a long form and a short form. */
    if (strcmp(command, alias1) == 0) {
        return 1U;
    }

    if (strcmp(command, alias2) == 0) {
        return 1U;
    }

    return 0U;
}

// this is the output line! so when you ask for a reading this will come up ina line for your viewing. 
static void PrintStatus(void)
{
    char msg[128];

    /* Build one readable line for PuTTY. */
    snprintf(msg, sizeof(msg),
             "Raw=%lu Moisture=%lu%% Pump=%s Auto=%s Cooldown=%s\r\n",
             (unsigned long)lastRaw,
             (unsigned long)lastPercent,
             (pumpActive != 0U) ? "ON" : "OFF",
             (autoMode != 0U) ? "ON" : "OFF",
             (cooldownActive != 0U) ? "YES" : "NO");

    UART_SendString(msg);
}
