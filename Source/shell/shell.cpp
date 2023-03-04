#include "shell.h"

#include "memory/flash.h"

#include <FreeRTOS.h>

#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>

// UART interface that is used by shell
#define SHELL_IFACE EVAL_COM1

#define DECLARE_SHELL_PROCEDURE(name) {#name, ShellProc_##name}

typedef int (*TShellProcHandler)(int argc, char *argv[]);

struct SShellProc
{
    char* name;
    TShellProcHandler handler;
};

// Process incoming byte with shell
// Execute a buffered command if the byte is a new-line character
void ShellRecv(char data);

#ifdef __cplusplus
extern "C"
{
#endif

void USART1_IRQHandler(void)
{
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

int ShellProc_interval(int argc, char *argv[])
{
    printf("ShellProc_interval\r\n");
    return 1;
}

int ShellProc_device_id(int argc, char *argv[])
{
    printf("ShellProc_device_id\r\n");
    return 1;
}

int ShellProc_help(int argc, char *argv[])
{
    printf("Application commands:\r\n");
    printf("\tinterval [set] [val]  - read/write ranging interval (5-20)\r\n");
    printf("\tdevice_id [set] [val] - read/write device unique id number\r\n");
    printf("\thelp                  - show list of commands\r\n");
    return 1;
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

    if (data == '\n')
    {
        ShellDispatchCmd(rxBuf);
        rxBuf.clear();
    }
    else if (data != '\r')
    {
        rxBuf += data;
    }
}
