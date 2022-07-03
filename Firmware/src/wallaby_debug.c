#include "wallaby_debug.h"
#include <stdint.h>
#include <stdarg.h>

#include "stm32f4xx.h"


void wallaby_debug_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

  // set up serial on UART3 to main processor
  // PB10 TX   PB11 RX
  USART_InitTypeDef usartConfig;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // RCC_APB1Periph_AFIO
  usartConfig.USART_BaudRate = 115200;
  usartConfig.USART_WordLength = USART_WordLength_8b;
  usartConfig.USART_StopBits = USART_StopBits_1;
  usartConfig.USART_Parity = USART_Parity_No;
  usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART3, &usartConfig);

  USART_Cmd(USART3, ENABLE);
}

typedef struct
{
  const char *command;
  void (*function)(const char *const);
} wallaby_debug_command;

static wallaby_debug_command commands[32];
static uint8_t num_commands = 0;

void wallaby_debug_register_command(const char *command, void (*function)(const char *const))
{
  if (num_commands >= 32) return;
  commands[num_commands].command = command;
  commands[num_commands].function = function;
  num_commands++;
}

void wallaby_debug_printf(const char *format, ...)
{
  char buffer[512];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  const int buffer_size = strlen(buffer);
  for (uint8_t i = 0; i < buffer_size; i++)
  {
    USART_SendData(USART3, (uint8_t)buffer[i]);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  }
}

static char incoming_buffer[256];
static uint8_t incoming_buffer_index = 0;

void wallaby_debug_poll(void)
{
  if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET) return;

  char c = USART_ReceiveData(USART3);

  if (c == 127 || c == 8)
  {
    // Backspace
    if (incoming_buffer_index > 0)
    {
      incoming_buffer_index--;
      USART_SendData(USART3, '\b');
    }
    return;
  }

  
  if (incoming_buffer_index > sizeof (incoming_buffer) - 1)
  {
    incoming_buffer_index = 0;
    return;
  }

  // Echo the character back to the terminal
  USART_SendData(USART3, c);
  if (c == '\r')
  {
    USART_SendData(USART3, '\n');
  }

  // Check if return and call appropriate command
  if (c == '\r')
  {
    incoming_buffer[incoming_buffer_index] = 0;

    // Find first space in command
    uint8_t command_index = 0;
    while (incoming_buffer[command_index] != 0 && incoming_buffer[command_index] != ' ') command_index++;

    uint8_t hit = 0;
    for (uint8_t i = 0; i < num_commands; i++)
    {
      if (strncmp(incoming_buffer, commands[i].command, command_index) != 0) continue;
      commands[i].function(incoming_buffer);
      hit = 1;
      break;
    }

    incoming_buffer_index = 0;
  
    if (!hit)
    {
      wallaby_debug_printf("Unknown command: %s\r\n", incoming_buffer);
    }
  }
  else
  {
    incoming_buffer[incoming_buffer_index] = c;
    incoming_buffer_index++;
  }
}