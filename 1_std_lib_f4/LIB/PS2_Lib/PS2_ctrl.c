#include "PS2_ctrl.h"

// PS2 Pin Config Definition
PS2_Pin PS2_Pin_;
// Pin Config for PS2

void PS2_Pin_Config(GPIO_TypeDef* PS2_GPIO, uint32_t PS2_CS_Pin, uint32_t PS2_CLK_Pin, uint32_t PS2_CMD_Pin, uint32_t PS2_DAT_Pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	(&PS2_Pin_)->PS2_GPIO = PS2_GPIO;
	(&PS2_Pin_)->PS2_CS_Pin = PS2_CS_Pin;
	(&PS2_Pin_)->PS2_CLK_Pin = PS2_CLK_Pin;
	(&PS2_Pin_)->PS2_CMD_Pin = PS2_CMD_Pin;
	(&PS2_Pin_)->PS2_DAT_Pin = PS2_DAT_Pin;

  // SPI CS pin configuration
  GPIO_InitStructure.GPIO_Pin = (&PS2_Pin_)->PS2_CS_Pin | (&PS2_Pin_)->PS2_CLK_Pin | (&PS2_Pin_)->PS2_CMD_Pin; // CSS - CLK - MOSI
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init((&PS2_Pin_)->PS2_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = (&PS2_Pin_)->PS2_DAT_Pin; // CLK - MISO - MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init((&PS2_Pin_)->PS2_GPIO, &GPIO_InitStructure);

}

// Analog
uint8_t Analog(uint8_t button) {
   return PS2_Data[button];
}

uint16_t Digital(void) {
		if (ButtonPressed(PSB_PAD_UP))
			return PSB_PAD_UP;
		if (ButtonPressed(PSB_PAD_RIGHT))
			return PSB_PAD_RIGHT;
		if (ButtonPressed(PSB_PAD_DOWN))
			return PSB_PAD_DOWN;
		if (ButtonPressed(PSB_PAD_LEFT))
			return PSB_PAD_LEFT;

		if (ButtonPressed(PSB_L2))
			return PSB_L2;
		if (ButtonPressed(PSB_R2))
			return PSB_R2;
		if (ButtonPressed(PSB_L1))
			return PSB_L1;
		if (ButtonPressed(PSB_R1))
			return PSB_R1;

		if (ButtonPressed(PSB_TRIANGLE))
			return PSB_TRIANGLE;
		if (ButtonPressed(PSB_CIRCLE))
			return PSB_CIRCLE;
		if (ButtonPressed(PSB_CROSS))
			return PSB_CROSS;
		if (ButtonPressed(PSB_SQUARE))
			return PSB_SQUARE;
		
		return 0;
}

// Read Buttons

uint8_t NewButtonState(unsigned int button)
{
	return (((last_buttons ^ buttons) & button) > 0);
}

uint8_t Button(uint16_t button)
{
	return ((~buttons & button) > 0);
}

uint8_t ButtonPressed(unsigned int button)
{
	return(NewButtonState(button) & Button(button));
}

// Data transfer
uint8_t _gamepad_shiftinout(uint8_t byte)
{

		 unsigned char tmp = 0;
		 int i;
		 for(i=0;i<8;i++) {
				if(CHK(byte,i)) CMD_SET();
				else CMD_CLR();
			
				CLK_CLR();
				vTaskDelay(1);

				//if(DAT_CHK()) SET(tmp,i);
				if(DAT_CHK()) tmp |= (1 << i);

				CLK_SET();
		 }
		 CMD_SET();
		 vTaskDelay(8);
		 return tmp;
		
}

//function for send data PS2

void PS2_readGamePad(void)
{
	int i;
	uint8_t dword[9] = {0x01,0x42,0,0,0,0,0,0,0};
  uint8_t dword2[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	vTaskDelay(10);
	while(1)
	{
		CMD_SET();
		CLK_SET();
		CS_LOW;
		for (i = 0; i < 9; i++)
		{
			PS2_Data[i] = _gamepad_shiftinout(dword[i]);
		}
		
		if(PS2_Data[1] == 0x79) {  //if controller is in full data return mode, get the rest of data
			 for (i = 0; i<12; i++) {
					PS2_Data[i+9] = _gamepad_shiftinout(dword2[i]);
			 }
		}
		CS_HIGH;
		
    if ((PS2_Data[1] & 0xf0) == 0x70)
			break;
    PS2_reconfigGamePad(); // try to get back into Analog mode.
    vTaskDelay(50);
	}
	last_buttons = buttons;
	buttons =  (uint16_t)(PS2_Data[4] << 8) + PS2_Data[3];
}

uint8_t PS2_configGamePad(void)
{

  PS2_readGamePad();

  if(PS2_Data[1] != 0x41 && PS2_Data[1] != 0x73 && PS2_Data[1] != 0x79){ 
    return 1; //return error code 1
  }
	
	PS2_sendCommandString(enter_config, sizeof(enter_config)); //start config run
	while(1)
	{
		CMD_SET();
		CLK_SET();
		CS_LOW;
		PS2_sendCommandString(enter_config, sizeof(enter_config)); //start config run
		vTaskDelay(10);
		
    PS2_sendCommandString(set_mode, sizeof(set_mode));
    PS2_sendCommandString(exit_config, sizeof(exit_config));
		PS2_readGamePad();
		
    if(PS2_Data[1] == 0x73)
      break;

		vTaskDelay(10);
	}
	return 0;
}

void PS2_sendCommandString(uint8_t string[], int length)
{
	int i = 0;
	CS_LOW; //set CS pin low to select PS2
	vTaskDelay(10);
	
	for(i = 0; i < length; i++)
	{
    _gamepad_shiftinout(string[i]);
	}
	vTaskDelay(10);
	
	// Return the byte read from the SPI bus
	CS_HIGH;
}

void PS2_reconfigGamePad(void)
{
	PS2_sendCommandString(enter_config, sizeof(enter_config)); //start config run
	PS2_sendCommandString(set_mode, sizeof(set_mode));
	PS2_sendCommandString(exit_config, sizeof(exit_config));
}
