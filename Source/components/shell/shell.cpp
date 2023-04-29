#include "shell.h"

#include "config/config.h"
#include "memory/flash.h"

#include <FreeRTOS.h>
#include <semphr.h>

#include <string>
#include <vector>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

// UART interface that is used by shell
#define SHELL_IFACE EVAL_COM1

#define SHELL_PROMPT "> "

#define DECLARE_SHELL_PROCEDURE(name) {#name, ShellProc_##name}

typedef void(*TShellProcHandler)(std::vector<std::string> argv);

struct SShellProc
{
    char* name;
    TShellProcHandler handler;
};

// Process incoming byte with shell
// Execute a buffered command if the byte is a new-line character
void ShellRecv(char data);

// Is shell enabled
// Commands and print via shell interface are ignored if shell is not enabled
// This allows use silent mode, when nothing is printed to and received from the console
static bool g_bShellInitialized = false;

// Shell mutex to prevent simultaneous print from different threads
static xSemaphoreHandle s_shellSemaphore = NULL;

#ifdef __cplusplus
extern "C"
{
#endif

void USART1_IRQHandler(void)
{
    if (!g_bShellInitialized)
    {
        return;
    }

    if(USART_GetITStatus(SHELL_IFACE, USART_IT_RXNE) != RESET)
    {
        // Read one byte from the receive data register
        char buff = USART_ReceiveData(SHELL_IFACE);
        ShellRecv(buff);
    }
}

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
    USART_ClearFlag(SHELL_IFACE, USART_FLAG_TC);
    /* Write a character to shell */
    USART_SendData(SHELL_IFACE, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(SHELL_IFACE, USART_FLAG_TC) == RESET)
    {}

    return ch;
}

#ifdef __cplusplus
}
#endif

void ShellProc_device_id(std::vector<std::string> argv)
{
    SRuntimeConfig cfg = ConfigRead();

    if (argv.size() > 1)
    {
        if (argv[1] == "set")
        {
            if (argv.size() > 2)
            {
                const int newDevId = atoi(argv[2].c_str());
                if (newDevId >= 0 && newDevId <= 255)
                {
                    cfg.deviceId = newDevId;
                    ConfigWrite(cfg);
                    printf("Device ID is set to '%d'\r\n", cfg.deviceId);
                }
                else
                {
                    printf("Invalid ID '%d', must be in range [0-255]\r\n", newDevId);
                }
            }
            else
            {
                printf("Device ID is not specified!\r\n");
            }
        }
        else
        {
            printf("Unknown arg '%s'!\r\n", argv[1].c_str());
        }
    }
    else
    {
        printf("Device ID: %d\r\n", cfg.deviceId);
    }
}

#if DWM_VAR_TRX_DLY
void ShellProc_ant_dly(std::vector<std::string> argv)
{
    SRuntimeConfig cfg = ConfigRead();

    if (argv.size() > 1)
    {
        if (argv[1] == "set")
        {
            if (argv.size() > 2)
            {
                const int newDelay = atoi(argv[2].c_str());
                if (newDelay >= 0)
                {
                    cfg.aggAntDelay = newDelay;
                    ConfigWrite(cfg);
                    printf("Antenna delay is set to '%d'\r\n", cfg.aggAntDelay);
                    printf("\tTX: %d\r\n", cfg.aggAntDelay / 2);
                    printf("\tRX: %d\r\n", cfg.aggAntDelay / 2);
                }
                else
                {
                    printf("Invalid delay '%d', must be greater than 0\r\n", newDelay);
                }
            }
            else
            {
                printf("Antenna delay is not specified!\r\n");
            }
        }
        else
        {
            printf("Unknown arg '%s'!\r\n", argv[1].c_str());
        }
    }
    else
    {
        printf("Antenna delay (TRX): %d\r\n", cfg.aggAntDelay);
        printf("\tTX: %d\r\n", cfg.aggAntDelay / 2);
        printf("\tRX: %d\r\n", cfg.aggAntDelay / 2);
    }
}
#endif // DWM_VAR_TRX_DLY

void ShellProc_reset(std::vector<std::string> argv)
{
    printf("HW reset initiated...\r\n");
    NVIC_SystemReset();
}

void ShellProc_help(std::vector<std::string> argv)
{
    printf("Application commands:\r\n");
    printf("\tdevice_id [set] [val] - read/write device unique id number [0-255]\r\n");
#if DWM_VAR_TRX_DLY
    printf("\tant_dly [set] [val] - read/write antenna delay value (sum of TX and RX)\r\n");
#endif // DWM_VAR_TRX_DLY
    printf("\treset                 - system hard reboot\r\n");
    printf("\thelp                  - show list of commands\r\n");
}

SShellProc shellProcedures[] =
{
    DECLARE_SHELL_PROCEDURE(device_id),
#if DWM_VAR_TRX_DLY
    DECLARE_SHELL_PROCEDURE(ant_dly),
#endif // DWM_VAR_TRX_DLY
    DECLARE_SHELL_PROCEDURE(reset),
    DECLARE_SHELL_PROCEDURE(help),
};

void ShellDispatchCmd(const std::string& cmd)
{
    std::vector<std::string> args;

    std::string arg = "";
    for (int i = 0; i <= cmd.size(); ++i)
    {
        if (i == cmd.size() || cmd[i] == ' ')
        {
            if (!arg.empty())
            {
                args.push_back(arg);
                arg.clear();
            }
        }
        else
        {
            arg += cmd[i];
        }
    }

    if (args.empty())
    {
        return;
    }

    bool bCmdFound = false;

    for (int i = 0; i < sizeof(shellProcedures) / sizeof(SShellProc); ++i)
    {
        if (args[0] == shellProcedures[i].name)
        {
            shellProcedures[i].handler(args);
            bCmdFound = true;
        }
    }

    if (!bCmdFound)
    {
        printf("Command '%s' doesn't exist! Use 'help' to see list of commands.\r\n", args[0].c_str());
    }
}

void ShellRecv(char data)
{
    static std::string rxBuf;

    // Echo
    putchar(data);

    // Handle \r\n sequence as an end of command (ignore \r and execute cmd at \n)
    switch (data)
    {
    // Backspace
    case 127:
    {
        if (!rxBuf.empty())
        {
            rxBuf = rxBuf.substr(0, rxBuf.size() - 1);
        }
        break;
    }
    case '\r':
    {
        break;
    }
    case '\n':
    {
        if (!rxBuf.empty())
        {
            ShellDispatchCmd(rxBuf);
            rxBuf.clear();
        }
        printf(SHELL_PROMPT);
        break;
    }
    default:
    {
        // Accumulate rx data in other cases
        rxBuf += data;
        break;
    }
    }
}

void ShellInit()
{
    if (!g_bShellInitialized)
    {
        g_bShellInitialized = true;
        s_shellSemaphore = xSemaphoreCreateMutex();
        configASSERT(s_shellSemaphore != NULL);

        printf(SHELL_PROMPT);
    }
}

int ShellPrintf(const char* format, ...)
{
    if (!g_bShellInitialized)
    {
        return 0;
    }

    if (xSemaphoreTake(s_shellSemaphore, portMAX_DELAY) != pdTRUE)
    {
        printf("[Shell] Could not take semaphore on printf!!\r\n");
        return 0;
    }

    va_list args;
    va_start(args, format);
    int len = vprintf(format, args);
    va_end(args);

    xSemaphoreGive(s_shellSemaphore);

    return len;
}
