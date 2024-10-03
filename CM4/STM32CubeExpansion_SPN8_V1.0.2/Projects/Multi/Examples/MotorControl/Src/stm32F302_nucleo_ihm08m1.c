/**
 ******************************************************************************
 * @file    stm32F302_nucleo_ihm08m1.c
 * @author  System lab - Automation and Motion control team
 * @version V1.1.0
 * @date    23-Oct-2015
 * @brief   This file provides the interface between the MC-lib and STM Nucleo
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


  #include "stm32F302_nucleo_ihm08m1.h"
  #include "6Step_Lib.h"

  extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters; /*!< Main SixStep structure*/ 
  extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters; /*!< SixStep PI regulator structure*/ 
  extern L6398_MotorDriver_TypeDef L6398MotorDriver;
  extern void MC_ADCx_SixStep_Bemf(void);
  extern void MC_TIMx_SixStep_timebase(void);
  extern void MC_SysTick_SixStep_MediumFrequencyTask(void);
  
  
/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup MC_6-STEP_LIB    MC_6-STEP_LIB
  * @brief  Motor Control driver
  * @{ 
  */

/** @addtogroup stm32F302_nucleo_ihm08m1    stm32F302_nucleo_ihm08m1
  * @brief  Interface file for STM32F302 and Motor Control Library configuration
  * @{ 
  */

/** @defgroup MC_SixStep_ADC_Channel    MC_SixStep_ADC_Channel
  *  @{
    * @brief Select the new ADC Channel 
*/
  void MC_SixStep_ADC_Channel(uint32_t adc_ch)
  {
     ADCx.Instance->CR |= ADC_CR_ADSTP;  
     while(ADCx.Instance->CR & ADC_CR_ADSTP);   
     /* Clear the old SQx bits for the selected rank */
     ADCx.Instance->SQR1 &= ~__HAL_ADC_SQR1_RK(ADC_SQR2_SQ5, 1);
     /* Set the SQx bits for the selected rank */
     ADCx.Instance->SQR1 |= __HAL_ADC_SQR1_RK(adc_ch, 1);
     ADCx.Instance->CR |= ADC_CR_ADSTART;  
  }

/**
  * @} 
  */

/** @defgroup MC_SixStep_Nucleo_Init    MC_SixStep_Nucleo_Init
  *  @{
    * @brief Init the STM32 register
*/

  void MC_SixStep_Nucleo_Init()
  {
    TIM_ClearInputConfigTypeDef sClearInputConfig;
    ADC_ChannelConfTypeDef sConfig;
 
if(!VOLTAGE_MODE)   
{    
    /******************** ETR CONFIGURATION ************************************/
    sClearInputConfig.ClearInputState = 1;  
    sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
    sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
    sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
    sClearInputConfig.ClearInputFilter = 0;
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH1);
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH2);
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH3);
    /***************************************************************************/
}  
    __HAL_FREEZE_TIM1_DBGMCU();  /* Stop TIM during Breakpoint*/
    
    __HAL_TIM_ENABLE_IT(&HF_TIMx, TIM_IT_BREAK); /* Enable the TIM Break interrupt */
    
    /******************** REGULAR CHANNELS CONFIGURATION *************************/
    sConfig.Channel = ADC_CH_1; /* Current feedabck */
    sConfig.Rank = 1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_CH_1_ST;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&ADCx, &sConfig);
    sConfig.Channel = ADC_CH_3; /* Bus voltage */
    sConfig.SamplingTime = ADC_CH_3_ST;
    HAL_ADC_ConfigChannel(&ADCx, &sConfig);
    sConfig.Channel = ADC_CH_4; /* Temperature feedback */
    sConfig.SamplingTime = ADC_CH_4_ST;
    HAL_ADC_ConfigChannel(&ADCx, &sConfig);
    sConfig.Channel = ADC_Bemf_CH1; /* BEMF feedback phase A */
    sConfig.SamplingTime = ADC_Bemf_CH1_ST;
    HAL_ADC_ConfigChannel(&ADCx, &sConfig);
    sConfig.Channel = ADC_Bemf_CH2; /* BEMF feedback phase B */
    sConfig.SamplingTime = ADC_Bemf_CH2_ST;
    HAL_ADC_ConfigChannel(&ADCx, &sConfig);
    sConfig.Channel = ADC_Bemf_CH3; /* BEMF feedback phase C */
    sConfig.SamplingTime = ADC_Bemf_CH3_ST;
    HAL_ADC_ConfigChannel(&ADCx, &sConfig);
    sConfig.Channel = ADC_CH_2; /* Potentiometer */
    sConfig.SamplingTime = ADC_CH_2_ST;
    HAL_ADC_ConfigChannel(&ADCx, &sConfig);    
    /***************************************************************************/
  }

/**
  * @} 
  */

/** @defgroup START_Ref_Generation    START_Ref_Generation
  *  @{
    * @brief Start the current generation
*/
  void START_Ref_Generation()
  {
     REFx.Instance->CCR1 = 0;
     HAL_TIM_PWM_Start(&REFx,TIM_CHANNEL_1);
  } 
/**
  * @} 
  */

/** @defgroup STOP_Ref_Generation    STOP_Ref_Generation
  *  @{
    * @brief Stop the current generation
*/
  void STOP_Ref_Generation()
  {
     REFx.Instance->CCR1 = 0;
     HAL_TIM_PWM_Stop(&REFx,TIM_CHANNEL_1);  
  }
/**
  * @} 
  */
/** @defgroup Set_Ref_Generation    Set_Ref_Generation
  *  @{
    * @brief Set the current generation value
*/
  void Set_Ref_Generation(uint16_t Iref)
  {
    if(!VOLTAGE_MODE)   
    {      
      REFx.Instance->CCR1 = (uint32_t)(Iref * REFx.Instance->ARR)/UPPER_OUT_LIMIT;
    }
    else SIXSTEP_parameters.pulse_value = Iref;     
  }
/**
  * @} 
  */
  
/** @defgroup START_DAC WITH PWM  START_DAC WITH PWM
  *  @{
     @brief Start DAC for debug based on PWM
*/
  void START_DAC()
  {
     DACx.Instance->CCR2 = 0;
     HAL_TIM_PWM_Start(&DACx, HF_TIMx_CH2); 
  } 
/**
  * @} 
  */  
/** @defgroup STOP_DAC    STOP_DAC
  *  @{
     @brief Stop DAC for debug
*/
  void STOP_DAC()
  {
     DACx.Instance->CCR2 = 0;
     HAL_TIM_PWM_Stop(&DACx, HF_TIMx_CH2);   
  }
/**
  * @} 
  */    
/** @defgroup SET_DAC_value    SET_DAC_value
  *  @{
     @brief Set DAC value for debug
*/
  void SET_DAC_value(uint16_t dac_value)
  {
      DACx.Instance->CCR2 = (uint32_t)(dac_value);
  }
/**
  * @} 
  */   

/** @defgroup HAL_ADC_ConvCpltCallback    HAL_ADC_ConvCpltCallback
  *  @{
    * @brief ADC callback 
*/
  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* ADCx)
  {
     MC_ADCx_SixStep_Bemf();
  }
/**
  * @} 
  */

/** @defgroup HAL_TIM_PeriodElapsedCallback    HAL_TIM_PeriodElapsedCallback
  *  @{
    * @brief HF htim callback 
*/
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *HF_TIMx)
  {
    MC_TIMx_SixStep_timebase();
  }
/**
  * @} 
  */

/** @defgroup HAL_SYSTICK_Callback    HAL_SYSTICK_Callback
  *  @{
    * @brief Systick callback 
*/
  void HAL_SYSTICK_Callback()
  {
    MC_SysTick_SixStep_MediumFrequencyTask();
  }
/**
  * @} 
  */

/** @defgroup HAL_GPIO_EXTI_Callback    HAL_GPIO_EXTI_Callback
  *  @{
    * @brief EXT callback 
*/ 
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
    MC_EXT_button_SixStep();
  }
/**
  * @} 
  */
  
/** @defgroup EnableInput_CH1_E_CH2_E_CH3_D    EnableInput_CH1_E_CH2_E_CH3_D
  *  @{
    * @brief Enable Input channel for L6398
  */

void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(uint8_t step_position,uint8_t synchronous_rect,uint16_t CCR_value)
{
   L6398MotorDriver.EnableInput_CH1_E_CH2_E_CH3_D(step_position,synchronous_rect,CCR_value);
}

/**
  * @} 
  */  
  
 /** @defgroup EnableInput_CH1_E_CH2_D_CH3_E    EnableInput_CH1_E_CH2_D_CH3_E
  *  @{
    * @brief Enable Input channel for L6398
  */
void  MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(uint8_t step_position,uint8_t synchronous_rect,uint16_t CCR_value)
{
  L6398MotorDriver.EnableInput_CH1_E_CH2_D_CH3_E(step_position,synchronous_rect,CCR_value);
}

/**
  * @} 
  */ 

/** @defgroup EnableInput_CH1_D_CH2_E_CH3_E    EnableInput_CH1_D_CH2_E_CH3_E
  *  @{
    * @brief Enable Input channel for L6398
  */
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(uint8_t step_position,uint8_t synchronous_rect,uint16_t CCR_value)
{
  L6398MotorDriver.EnableInput_CH1_D_CH2_E_CH3_E(step_position,synchronous_rect,CCR_value);
}

/**
  * @} 
  */

/** @defgroup DisableInput_CH1_D_CH2_D_CH3_D    DisableInput_CH1_D_CH2_D_CH3_D
  *  @{
    * @brief Disable All Input channels for L6398
  */

void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D()
{
  L6398MotorDriver.DisableInput_CH1_D_CH2_D_CH3_D();
}

/**
  * @} 
  */

/** @defgroup Start_PWM_driving    Start_PWM_driving
  *  @{
    * @brief Enable the PWM generation on Input channels
  */

void MC_SixStep_Start_PWM_driving()
{
   L6398MotorDriver.Start_PWM_driving();
   HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH1);
   HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH1); 
   HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH2); 
   HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH2);    
   HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH3); 
   HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH3);    
} 

/**
  * @} 
  */

/** @defgroup Stop_PWM_driving    Stop_PWM_driving
  *  @{
    * @brief Disable the PWM generation on Input channels
  */

void MC_SixStep_Stop_PWM_driving()
{
  L6398MotorDriver.Stop_PWM_driving();
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH1);
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH1); 
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH2); 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH2);    
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH3); 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH3);     
}  

/**
  * @}
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH1    HF_TIMx_SetDutyCycle_CH1
  *  @{
    * @brief Set the Duty Cycle value for CH1 
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
  L6398MotorDriver.HF_TIMx_SetDutyCycle_CH1(CCR_value);
}


/**
  * @} 
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH2    HF_TIMx_SetDutyCycle_CH2
  *  @{
    * @brief Set the Duty Cycle value for CH2 
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
  L6398MotorDriver.HF_TIMx_SetDutyCycle_CH2(CCR_value);
}
/**
  * @} 
  */


/** @defgroup HF_TIMx_SetDutyCycle_CH3    HF_TIMx_SetDutyCycle_CH3
  *  @{
    * @brief Set the Duty Cycle value for CH3 
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
  L6398MotorDriver.HF_TIMx_SetDutyCycle_CH3(CCR_value);
}

/**
  * @} 
  */

/** @defgroup Current_Reference_Start    Current_Reference_Start
  *  @{
    * @brief Enable the Current Reference generation
*/

void MC_SixStep_Current_Reference_Start()
{
  L6398MotorDriver.Current_Reference_Start();
}

/**
  * @} 
  */


/** @defgroup Current_Reference_Stop    Current_Reference_Stop
  *  @{
    * @brief Disable the Current Reference generation
*/

void MC_SixStep_Current_Reference_Stop()
{
  L6398MotorDriver.Current_Reference_Stop();
}

/**
  * @}  
  */


/** @defgroup Current_Reference_Setvalue    Current_Reference_Setvalue
  *  @{
    * @brief Set the value for Current Reference
*/

void MC_SixStep_Current_Reference_Setvalue(uint16_t Iref)
{
  L6398MotorDriver.Current_Reference_Setvalue(Iref);
}

/**
  * @}  
  */


/** @defgroup Get_UART_data   Get_UART_data
  *  @{
    * @brief Get the UART value from DR register
*/
uint32_t Get_UART_Data()
{
  return (UART.Instance->RDR);
}
/**
 * @}  
 */ 

/**
  * @} // end STM32F302_Interface
  */

/**
  * @}  end MC_6-STEP_LIB 
  */
    
/**
  * @} // end MIDDLEWARES
  */
