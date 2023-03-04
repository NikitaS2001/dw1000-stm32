#include "shell.h"

#include "memory/flash.h"

#include <FreeRTOS.h>

#include <string>
#include <vector>

#include <stdio.h>

// UART interface that is used by shell
#define SHELL_IFACE EVAL_COM1

#define SHELL_PROMPT "> "

#define DECLARE_SHELL_PROCEDURE(name) {#name, ShellProc_##name}

typedef void(*TShellProcHandler)(int argc, char *argv[]);

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
    if (!g_bShellInitialized)
    {
        return ch;
    }

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

void ShellProc_interval(int argc, char *argv[])
{
    printf("ShellProc_interval\r\n");
}

void ShellProc_device_id(int argc, char *argv[])
{
    printf("ShellProc_device_id\r\n");
}

void ShellProc_help(int argc, char *argv[])
{
    printf("Application commands:\r\n");
    printf("\tinterval [set] [val]  - read/write ranging interval (5-20)\r\n");
    printf("\tdevice_id [set] [val] - read/write device unique id number\r\n");
    printf("\thelp                  - show list of commands\r\n");
}

SShellProc shellProcedures[] =
{
    DECLARE_SHELL_PROCEDURE(interval),
    DECLARE_SHELL_PROCEDURE(device_id),
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
            shellProcedures[i].handler(0, 0);
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
        printf(SHELL_PROMPT);
    }
}
