#ifndef BASIC_STMF4
#define BASIC_STMF4
	#include "stm32f4xx.h"
	#include "FreeRTOS.h"
	#include "task.h"
	#include "queue.h"
	#include "list.h"
#endif  /* BASIC_STMF4 */

#ifndef PS2_CTRL
#define PS2_CTRL

	#define PSB_PAD_UP      0x0010
	#define PSB_PAD_RIGHT   0x0020
	#define PSB_PAD_DOWN    0x0040
	#define PSB_PAD_LEFT    0x0080

	#define PSB_L2          0x0100
	#define PSB_R2          0x0200
	#define PSB_L1          0x0400
	#define PSB_R1          0x0800

	#define PSB_TRIANGLE    0x1000
	#define PSB_CIRCLE      0x2000
	#define PSB_CROSS       0x4000
	#define PSB_SQUARE      0x8000

	#define PSS_RX 5
	#define PSS_RY 6
	#define PSS_LX 7
	#define PSS_LY 8

	#define CHK(x,y) (x & (1<<y))
	#define CMD_SET() GPIO_WriteBit((&PS2_Pin_)->PS2_GPIO, (&PS2_Pin_)->PS2_CMD_Pin, Bit_SET) 
	#define CMD_CLR() GPIO_WriteBit((&PS2_Pin_)->PS2_GPIO, (&PS2_Pin_)->PS2_CMD_Pin, Bit_RESET) 
	#define CLK_SET() GPIO_WriteBit((&PS2_Pin_)->PS2_GPIO, (&PS2_Pin_)->PS2_CLK_Pin, Bit_SET) 
	#define CLK_CLR() GPIO_WriteBit((&PS2_Pin_)->PS2_GPIO, (&PS2_Pin_)->PS2_CLK_Pin, Bit_RESET) 
	#define DAT_CHK() GPIO_ReadInputDataBit((&PS2_Pin_)->PS2_GPIO, (&PS2_Pin_)->PS2_DAT_Pin)

	#define CS_HIGH      GPIO_WriteBit((&PS2_Pin_)->PS2_GPIO, (&PS2_Pin_)->PS2_CS_Pin, Bit_SET)   
	#define CS_LOW       GPIO_WriteBit((&PS2_Pin_)->PS2_GPIO, (&PS2_Pin_)->PS2_CS_Pin, Bit_RESET)   

	typedef struct
	{
	 uint32_t PS2_CS_Pin;
	 uint32_t PS2_CLK_Pin;
	 uint32_t PS2_CMD_Pin;
	 uint32_t PS2_DAT_Pin;
	 GPIO_TypeDef* PS2_GPIO;		
	}PS2_Pin;

	// Initial PS2 Controller
	static uint8_t enter_config[]={0x01,0x43,0x00,0x01,0x00};
	static uint8_t set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
	static uint8_t exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
	static uint8_t type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};
	static uint8_t PS2_Data[21];

	static uint16_t buttons, last_buttons;

	void PS2_Pin_Config(GPIO_TypeDef* PS2_GPIO, uint32_t PS2_CS_Pin, uint32_t PS2_CLK_Pin, uint32_t PS2_CMD_Pin, uint32_t PS2_DAT_Pin);
	void PS2_readGamePad(void);
	uint8_t PS2_configGamePad(void);
	void PS2_reconfigGamePad(void);
	void PS2_sendCommandString(uint8_t commands[], int length);
	uint8_t _gamepad_shiftinout(uint8_t data);

	uint8_t Analog(uint8_t button);
	uint16_t Digital(void);
	
	uint8_t NewButtonState(unsigned int button);
	uint8_t Button(uint16_t button);
	uint8_t ButtonPressed(unsigned int button);

#endif  /* PS2_CTRL */
