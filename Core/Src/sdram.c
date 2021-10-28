 /***
	************************************************************************************************
	*	@file  	sdram.c
	*	@version V1.0
	*  @date    2020-3-2
	*	@author  ���ͿƼ�
	*	@brief   sdram��غ���
   ************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32F429BIT6���İ� ���ͺţ�FK429M1��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> �ļ�˵����
	*
	*  1.�����õ��ĺ��������� CubeMX ���汾5.3.0�����ɣ�������ֲ�ڹٷ�������Ĵ���
	*	2.FK429M1���İ�ʹ�õ�SDRAMΪ32λ��16M�ֽڴ�С
	*
	************************************************************************************************
***/

#include "sdram.h"   

SDRAM_HandleTypeDef hsdram2;			// SDRAM_HandleTypeDef �ṹ�����
FMC_SDRAM_CommandTypeDef command;	// ����ָ��

/*************************0*****************************************************************************
*	�� �� ��: SDRAM_delay
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: ������ʱ��������λԼΪ5ms
*	˵    ��: ��
*******************************************************************************************************/

void SDRAM_delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
	
  for(index = (100000 * nCount); index != 0; index--);

}

/*************************************************************************************************
*	�� �� ��:	HAL_FMC_MspInit
*	��ڲ���:	��
*	�� �� ֵ:��
*	��������:	��ʼ��sdram����
*	˵    ��:�ú����Ĵ����� CubeMX ���汾5.3.0������
*************************************************************************************************/

void HAL_FMC_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct ={0};

	__HAL_RCC_FMC_CLK_ENABLE();		// ����FMCʱ��

	__HAL_RCC_GPIOI_CLK_ENABLE();		// ����GPIOʱ��
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

  /** FMC GPIO Configuration  

	PD14   ------> FMC_D0			  PF0    ------> FMC_A0			  	PC0    ------> FMC_SDNWE
	PD15   ------> FMC_D1	        PF1    ------> FMC_A1          PF11   ------> FMC_SDNRAS
	PD0    ------> FMC_D2           PF2    ------> FMC_A2          PH6    ------> FMC_SDNE1
	PD1    ------> FMC_D3	        PF3    ------> FMC_A3          PH7    ------> FMC_SDCKE1	
	PE7    ------> FMC_D4           PF4    ------> FMC_A4          PG4    ------> FMC_BA0
	PE8    ------> FMC_D5           PF5    ------> FMC_A5          PG5    ------> FMC_BA1  
	PE9    ------> FMC_D6           PF12   ------> FMC_A6          PG8    ------> FMC_SDCLK 
	PE10   ------> FMC_D7           PF13   ------> FMC_A7          PG15   ------> FMC_SDNCAS
	PE11   ------> FMC_D8           PF14   ------> FMC_A8          PE0    ------> FMC_NBL0
	PE12   ------> FMC_D9           PF15   ------> FMC_A9          PE1    ------> FMC_NBL1
	PE13   ------> FMC_D10          PG0    ------> FMC_A10         PI4    ------> FMC_NBL2
	PE14   ------> FMC_D11          PG1    ------> FMC_A11         PI5    ------> FMC_NBL3	
	PE15   ------> FMC_D12
	PD8    ------> FMC_D13
	PD9    ------> FMC_D14
	PD10   ------> FMC_D15
	PH8    ------> FMC_D16
	PH9    ------> FMC_D17
	PH10   ------> FMC_D18
	PH11   ------> FMC_D19
	PH12   ------> FMC_D20 
	PH13   ------> FMC_D21
	PH14   ------> FMC_D22
	PH15   ------> FMC_D23  
	PI0    ------> FMC_D24
	PI1    ------> FMC_D25
	PI2    ------> FMC_D26
	PI3    ------> FMC_D27  
	PI6    ------> FMC_D28
	PI7    ------> FMC_D29	
	PI9    ------> FMC_D30
	PI10   ------> FMC_D31
 
 */
	GPIO_InitStruct.Pin 	= GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_0|GPIO_PIN_1 
								  |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
								  |GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 	= GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
								  |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12 
								  |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 			= GPIO_PIN_0;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
								  |GPIO_PIN_8|GPIO_PIN_15;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
								  |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
								  |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
								  |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
								  |GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
								  |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/*************************************************************************************************
*	�� �� ��:	HAL_SDRAM_MspInit
*	��ڲ���:	hsdram - SDRAM_HandleTypeDef����ı���������ʾ�����sdram
*	�� �� ֵ:��
*	��������:	��ʼ��sdram���ţ��ں��� HAL_SDRAM_Init �б�����
*	˵    ��:��		
*************************************************************************************************/

void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef* hsdram)
{
	HAL_FMC_MspInit();
}

/******************************************************************************************************
*	�� �� ��: SDRAM_Initialization_Sequence
*	��ڲ���: hsdram - SDRAM_HandleTypeDef����ı���������ʾ�����sdram
*				 Command	- ����ָ��
*	�� �� ֵ: ��
*	��������: SDRAM ��������
*	˵    ��: ����SDRAM���ʱ��Ϳ��Ʒ�ʽ
*******************************************************************************************************/

void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
  __IO uint32_t tmpmrd = 0;
  
  /* Configure a clock configuration enable command */
  Command->CommandMode 					= FMC_SDRAM_CMD_CLK_ENABLE;	// ����SDRAMʱ�� 
  Command->CommandTarget 				= FMC_COMMAND_TARGET_BANK; 	// ѡ��Ҫ���Ƶ�����
  Command->AutoRefreshNumber 			= 1;
  Command->ModeRegisterDefinition 	= 0;
  
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);	// ���Ϳ���ָ��
  SDRAM_delay(1);		// ��ʱ�ȴ�
  
  /* Configure a PALL (precharge all) command */ 
  Command->CommandMode 					= FMC_SDRAM_CMD_PALL;		// Ԥ�������
  Command->CommandTarget 				= FMC_COMMAND_TARGET_BANK;	// ѡ��Ҫ���Ƶ�����
  Command->AutoRefreshNumber 			= 1;
  Command->ModeRegisterDefinition 	= 0;
  
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);  // ���Ϳ���ָ��
  
  /* Configure a Auto-Refresh command */ 
  Command->CommandMode 					= FMC_SDRAM_CMD_AUTOREFRESH_MODE;	// ʹ���Զ�ˢ��
  Command->CommandTarget 				= FMC_COMMAND_TARGET_BANK;          // ѡ��Ҫ���Ƶ�����
  Command->AutoRefreshNumber			= 8;                                // �Զ�ˢ�´���
  Command->ModeRegisterDefinition 	= 0;
  
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);	// ���Ϳ���ָ��
  
  /* Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  
  Command->CommandMode					= FMC_SDRAM_CMD_LOAD_MODE;	// ����ģʽ�Ĵ�������
  Command->CommandTarget 				= FMC_COMMAND_TARGET_BANK;	// ѡ��Ҫ���Ƶ�����
  Command->AutoRefreshNumber 			= 1;
  Command->ModeRegisterDefinition 	= tmpmrd;
  
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);	// ���Ϳ���ָ��
  
  hsdram->Instance->SDRTR |= ((uint32_t)((1386)<< 1));	// ����ˢ�¼����� 
}

/******************************************************************************************************
*	�� �� ��: MX_FMC_Init
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: SDRAM��ʼ��
*	˵    ��: ��ʼ��FMC��SDRAM����
*******************************************************************************************************/

void MX_FMC_Init(void)
{

	FMC_SDRAM_TimingTypeDef SdramTiming = {0};

	hsdram2.Instance = FMC_SDRAM_DEVICE;

	hsdram2.Init.SDBank 						= FMC_SDRAM_BANK2;							// ѡ��BANK��
	hsdram2.Init.ColumnBitsNumber 		= FMC_SDRAM_COLUMN_BITS_NUM_8;         // �е�ַ���
	hsdram2.Init.RowBitsNumber 			= FMC_SDRAM_ROW_BITS_NUM_12;           // �е�ַ�߿��
	hsdram2.Init.MemoryDataWidth 			= FMC_SDRAM_MEM_BUS_WIDTH_32;          // ���ݿ��  
	hsdram2.Init.InternalBankNumber 		= FMC_SDRAM_INTERN_BANKS_NUM_4;        // bank����
	hsdram2.Init.CASLatency 				= FMC_SDRAM_CAS_LATENCY_3;             //	CAS 
	hsdram2.Init.WriteProtection 			= FMC_SDRAM_WRITE_PROTECTION_DISABLE;  // ��ֹд����
	hsdram2.Init.SDClockPeriod 			= FMC_SDRAM_CLOCK_PERIOD_2;            // ��Ƶ
	hsdram2.Init.ReadBurst 					= FMC_SDRAM_RBURST_ENABLE;             // ͻ��ģʽ  
	hsdram2.Init.ReadPipeDelay 			= FMC_SDRAM_RPIPE_DELAY_1;             // ���ӳ�

	SdramTiming.LoadToActiveDelay 		= 2;		// TMRD: min=12ns (2x11.11ns)  
	SdramTiming.ExitSelfRefreshDelay 	= 7;     // TXSR: min=72ns (7x11.11ns)  
	SdramTiming.SelfRefreshTime			= 4;     // TRAS: min=42ns (4x11.11ns)    
	SdramTiming.RowCycleDelay 				= 7;     // TRC:  min=60ns (7x11.11ns)   
	SdramTiming.WriteRecoveryTime 		= 2;     // TWR:  2 Tck 
	SdramTiming.RPDelay 						= 2;     // TRP:  18ns => 2x11.11ns       
	SdramTiming.RCDDelay 					= 2;     // TRCD: 12ns => 2x11.11ns 

	HAL_SDRAM_Init(&hsdram2, &SdramTiming);		// ��ʼ��FMC�ӿ�
															
	SDRAM_Initialization_Sequence(&hsdram2,&command);//����SDRAM
}

/******************************************************************************************************
*	�� �� ��: SDRAM_Test
*	��ڲ���: ��
*	�� �� ֵ: SUCCESS - �ɹ���ERROR - ʧ��
*	��������: SDRAM����
*	˵    ��: ����32λ�����ݿ��д�����ݣ��ٶ�ȡ����һһ���бȽϣ������8λ�����ݿ��д�룬
*				 ������֤NBL0��NBL1�������ŵ������Ƿ�������        
*******************************************************************************************************/

uint8_t SDRAM_Test(void)
{
	uint32_t i = 0;		// ��������
	uint32_t ReadData = 0; 	// ��ȡ��������
	uint8_t  ReadData_8b;
	
	printf("STM32F429 SDRAM����\r\n");
	printf("���Կ�ʼ����32λ���ݿ��д������...\r\n");	
	for (i = 0; i < SDRAM_Size/4; i++)
	{
 		*(__IO uint32_t*) (SDRAM_BANK_ADDR + 4*i) = i;		// д������
	}
	
	printf("д����ϣ���ȡ���ݲ��Ƚ�...\r\n");
	for(i = 0; i < SDRAM_Size/4;i++ )
	{
		ReadData = *(__IO uint32_t*)(SDRAM_BANK_ADDR + 4 * i );  // ��SDRAM��������	
		if( ReadData != i )      //������ݣ�������ȣ���������,���ؼ��ʧ�ܽ����
		{
			printf("SDRAM����ʧ�ܣ���\r\n");
			return ERROR;	 // ����ʧ�ܱ�־
		}
	}
	
	printf("32λ���ݿ�ȶ�дͨ������8λ���ݿ��д������\r\n");
	for (i = 0; i < 255; i++)
	{
 		*(__IO uint8_t*) (SDRAM_BANK_ADDR + i) = i;
	}	
	printf("д����ϣ���ȡ���ݲ��Ƚ�...\r\n");
	for (i = 0; i < 255; i++)
	{
		ReadData_8b = *(__IO uint8_t*) (SDRAM_BANK_ADDR + i);
		if( ReadData_8b != (uint8_t)i )      //������ݣ�������ȣ���������,���ؼ��ʧ�ܽ����
		{
			printf("8λ���ݿ�ȶ�д����ʧ�ܣ���\r\n");
			printf("����NBL0��NBL1������\r\n");	
			return ERROR;	 // ����ʧ�ܱ�־
		}
	}		
	printf("8λ���ݿ�ȶ�дͨ��\r\n");
	printf("SDRAM��д����ͨ����ϵͳ����\r\n");
	return SUCCESS;	 // ���سɹ���־
}

