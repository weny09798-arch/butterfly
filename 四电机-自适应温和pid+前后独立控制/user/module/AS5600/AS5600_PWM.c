#include "AS5600_PWM.h"

/**
************************************************************************************************
* @brief    AS5600_PWM角度读取
* @param    None
* @return   None
* @author   创源启明		2025.11.06
************************************************************************************************
**/

void AS5600_PWM_Init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1);
}


uint16_t AD_Value[4];

HAL_StatusTypeDef hhStatue;
void StarAndGetResult(void){

	hhStatue=HAL_ADC_Start_DMA(&hadc1,(uint32_t*)AD_Value,4);
	if(hhStatue!=HAL_OK)
	{
		AD_Value[0]=0;
		AD_Value[2]=0;
		
		AD_Value[1]=0;
		AD_Value[3]=0;
	}
	while (hdma_adc1.State!=HAL_DMA_STATE_READY);
	HAL_ADC_Stop_DMA(&hadc1);
	Wings_Data.Wings_motor[0].Corrective_Angle = PROCESS_VALUE(MAX_VALUE - AD_Value[0],MOTOR1_MIDPOINT);
	Wings_Data.Wings_motor[2].Corrective_Angle = PROCESS_VALUE(MAX_VALUE - AD_Value[2],MOTOR3_MIDPOINT);
	
	Wings_Data.Wings_motor[1].Corrective_Angle = PROCESS_VALUE(MAX_VALUE - AD_Value[1],MOTOR2_MIDPOINT);
	Wings_Data.Wings_motor[3].Corrective_Angle = PROCESS_VALUE(MAX_VALUE - AD_Value[3],MOTOR4_MIDPOINT);
	
}





