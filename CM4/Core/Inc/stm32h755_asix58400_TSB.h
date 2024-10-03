/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32H755_ASIX58400_TSB_H
#define STM32H755_ASIX58400_TSB_H

#define STM32H755

#define ADC_VER_V5_V90

  #include "stm32h7xx_hal.h"
  #include "main.h"
  #include "X-NUCLEO-IHM08M1.h"

  #define HF_TIMx               htim1 //advance timer
  #define LF_TIMx               htim6 //basic timer
  #define HALL_ENCODER_TIMx     htim2
//  #define ADCx                  hadc1
  #define REFx                  htim16
//  #define UART                  huart2

  #define GPIO_PORT_2           GPIOC
  #define GPIO_CH2              GPIO_PIN_11
  #define GPIO_PORT_3           GPIOC
  #define GPIO_CH3              GPIO_PIN_12
  #define GPIO_PORT_BEMF        GPIOC
  #define GPIO_CH_BEMF          GPIO_PIN_9

//  #define ADC_CH_1              ADC_CHANNEL_11    /*CURRENT*/ //PC1 ADC1
//  #define ADC_CH_2              ADC_CHANNEL_4    /*SPEED*/  // PC4 Speed ADC1
//  #define ADC_CH_3              ADC_CHANNEL_17    /*VBUS*/   //PA1 VBUS_Sensing ADC1
//  #define ADC_CH_4              ADC_CHANNEL_0 //ADC3   /*TEMP*/	//temperature feedback
//  #define ADC_Bemf_CH1          ADC_CHANNEL_1 //ADC3    /*BEMF1*/	//F302 - PC3
//  #define ADC_Bemf_CH2          ADC_CHANNEL_4 //ADC1  /*BEMF2*/	//F302 - PB11
//  #define ADC_Bemf_CH3          ADC_CHANNEL_8  //ADC1 /*BEMF3*/	//F302 - INP8 PC5
//
//  #define ADC_CH_1_ST           ADC_SAMPLETIME_1CYCLE_5    /*CURRENT sampling time */
//  #define ADC_CH_2_ST           ADC_SAMPLETIME_64CYCLES_5 /*SPEED sampling time*/
//  #define ADC_CH_3_ST           ADC_SAMPLETIME_64CYCLES_5 /*VBUS sampling time*/
//  #define ADC_CH_4_ST           ADC3_SAMPLETIME_92CYCLES_5 /*TEMP sampling time*/
//  #define ADC_Bemf_CH1_ST       ADC3_SAMPLETIME_47CYCLES_5  /*BEMF1 sampling time*/
//  #define ADC_Bemf_CH2_ST       ADC_SAMPLETIME_32CYCLES_5  /*BEMF2 sampling time*/
//  #define ADC_Bemf_CH3_ST       ADC_SAMPLETIME_32CYCLES_5  /*BEMF3 sampling time*/

  #define HF_TIMx_CH1           TIM_CHANNEL_1
  #define HF_TIMx_CH2           TIM_CHANNEL_2
  #define HF_TIMx_CH3           TIM_CHANNEL_3
  #define HF_TIMx_CCR1          CCR1            /*Channel 1*/
  #define HF_TIMx_CCR2          CCR2            /*Channel 2*/
  #define HF_TIMx_CCR3          CCR3            /*Channel 3*/

//check values for h7
//  #define DAC_ENABLE            1              /*!< Enable (1) the DAC peripheral */
//  #define DACx                  htim2
//  #define DACx_CH               DAC1_CHANNEL_1  /*!<  DAC Channel */
//  #define DACx_ALIGN            DAC_ALIGN_12B_L  /*!< DAC Aligment value */

  #define GPIO_PORT_ZCR         GPIOC           /*!<  GPIO port name for zero crossing detection */
  #define GPIO_CH_ZCR           GPIO_PIN_7      /*!<  GPIO pin name for zero crossing detection */
  #define GPIO_PORT_COMM        GPIOC           /*!<  GPIO port name for 6Step commutation */
  #define GPIO_CH_COMM          GPIO_PIN_10      /*!<  GPIO pin name for 6Step commutation */

  #define STARTM_CMD             0     /*!<  Start Motor command received */
  #define STOPMT_CMD             1     /*!<  Stop Motor command received */
  #define SETSPD_CMD             2     /*!<  Set the new speed value command received */
  #define GETSPD_CMD             3     /*!<  Get Mechanical Motor Speed command received */
  #define INIREF_CMD             4     /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */
  #define POLESP_CMD             5     /*!<  Set the Pole Pairs value command received */
  #define ACCELE_CMD             6     /*!<  Set the Accelleration for Start-up of the motor command received */
  #define KP_PRM_CMD             7    /*!<  Set the KP PI param command received */
  #define KI_PRM_CMD             8    /*!<  Set the KI PI param command received */
  #define POTENZ_CMD             9    /*!<  Enable Potentiometer command received */
  #define HELP_CMD               10    /*!<  Help command received */
  #define STATUS_CMD             11    /*!<  Get the Status of the system command received */
  #define DIRECT_CMD             12    /*!<  Get the motor direction */

  /** @addtogroup stm32F302_nucleo_ihm08m1    stm32F302_nucleo_ihm08m1
  * @brief  Interface file for STM32F302 and Library configuration
  * @{
  */

  /** @defgroup Exported_function_F302  Exported_function_F302
  * @{
  */
  /**
    * @brief  API function for STM32 instruction
    */
  uint32_t Get_UART_Data(void);
  void MC_SixStep_ADC_Channel(uint32_t);
  void MC_SixStep_Nucleo_Init(void);
  void START_Ref_Generation(void);
  void STOP_Ref_Generation(void);
  void Set_Ref_Generation(uint16_t);
  void START_DAC(void);
  void STOP_DAC(void);
  void SET_DAC_value(uint16_t);
  void Bemf_delay_calc(void);
  void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(uint8_t,uint8_t,uint16_t);
  void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(uint8_t,uint8_t,uint16_t);
  void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(uint8_t,uint8_t,uint16_t);
  void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D(void);
  void MC_SixStep_Start_PWM_driving(void);
  void MC_SixStep_Stop_PWM_driving(void);
  void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t);
  void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t);
  void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t);
  void MC_SixStep_Current_Reference_Start(void);
  void MC_SixStep_Current_Reference_Stop(void);
  void MC_SixStep_Current_Reference_Setvalue(uint16_t);

  /**
  * @}
  */

  /**
  * @}
  */

#endif
