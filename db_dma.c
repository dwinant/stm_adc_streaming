/*
 * db_dma.c
 *
 *  Created on: Dec 30, 2020
 *      Author: david.winant
 *
 *  Below is the original header from stm32f7xx_hal_adc.c
 *  from "STM32Cube MCU Package for STM32F7 Series"
 *  from which I derived the Start_DB_DMA function.
 *
 *  This module does not support callback registration or the USE_HAL_ADC_REGISTER_CALLBACKS flag.
 *
 *  (@) Callback functions must be implemented in user program:
 *   (+@) HAL_ADC_ErrorCallback()
 *   (+@) HAL_ADC_DB_ConvCpltCallback() (buffer conversion complete)
 *
 *  The HAL_ADC_DB_ConvCpltCallback gets a parameter which is set to 0 when the first
 *  buffer has been filled, and set to 1 to indicate the second buffer has been filled.
 *
 */
/**
  ******************************************************************************
  * @file    stm32f7xx_hal_adc.c
  * @author  MCD Application Team
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the Analog to Digital Convertor (ADC) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + State and errors functions
  *
  @verbatim
  ==============================================================================
                    ##### ADC Peripheral features #####
  ==============================================================================
  [..]
  (#) 12-bit, 10-bit, 8-bit or 6-bit configurable resolution.
  (#) Interrupt generation at the end of conversion, end of injected conversion,
      and in case of analog watchdog or overrun events
  (#) Single and continuous conversion modes.
  (#) Scan mode for automatic conversion of channel 0 to channel x.
  (#) Data alignment with in-built data coherency.
  (#) Channel-wise programmable sampling time.
  (#) External trigger option with configurable polarity for both regular and
      injected conversion.
  (#) Dual/Triple mode (on devices with 2 ADCs or more).
  (#) Configurable DMA data storage in Dual/Triple ADC mode.
  (#) Configurable delay between conversions in Dual/Triple interleaved mode.
  (#) ADC conversion type (refer to the datasheets).
  (#) ADC supply requirements: 2.4 V to 3.6 V at full speed and down to 1.8 V at
      slower speed.
  (#) ADC input range: VREF(minus) = VIN = VREF(plus).
  (#) DMA request generation during regular channel conversion.


                     ##### How to use this driver #####
  ==============================================================================
  [..]
  (#)Initialize the ADC low level resources by implementing the HAL_ADC_MspInit():
       (##) Enable the ADC interface clock using __HAL_RCC_ADC_CLK_ENABLE()
       (##) ADC pins configuration
             (+++) Enable the clock for the ADC GPIOs using the following function:
                   __HAL_RCC_GPIOx_CLK_ENABLE()
             (+++) Configure these ADC pins in analog mode using HAL_GPIO_Init()
       (##) In case of using interrupts (e.g. HAL_ADC_Start_IT())
             (+++) Configure the ADC interrupt priority using HAL_NVIC_SetPriority()
             (+++) Enable the ADC IRQ handler using HAL_NVIC_EnableIRQ()
             (+++) In ADC IRQ handler, call HAL_ADC_IRQHandler()
       (##) In case of using DMA to control data transfer (e.g. HAL_ADC_Start_DMA())
             (+++) Enable the DMAx interface clock using __HAL_RCC_DMAx_CLK_ENABLE()
             (+++) Configure and enable two DMA streams stream for managing data
                 transfer from peripheral to memory (output stream)
             (+++) Associate the initialized DMA handle to the CRYP DMA handle
                 using  __HAL_LINKDMA()
             (+++) Configure the priority and enable the NVIC for the transfer complete
                 interrupt on the two DMA Streams. The output stream should have higher
                 priority than the input stream.

    *** Configuration of ADC, groups regular/injected, channels parameters ***
  ==============================================================================
  [..]
  (#) Configure the ADC parameters (resolution, data alignment, ...)
      and regular group parameters (conversion trigger, sequencer, ...)
      using function HAL_ADC_Init().

  (#) Configure the channels for regular group parameters (channel number,
      channel rank into sequencer, ..., into regular group)
      using function HAL_ADC_ConfigChannel().

  (#) Optionally, configure the injected group parameters (conversion trigger,
      sequencer, ..., of injected group)
      and the channels for injected group parameters (channel number,
      channel rank into sequencer, ..., into injected group)
      using function HAL_ADCEx_InjectedConfigChannel().

  (#) Optionally, configure the analog watchdog parameters (channels
      monitored, thresholds, ...) using function HAL_ADC_AnalogWDGConfig().

  (#) Optionally, for devices with several ADC instances: configure the
      multimode parameters using function HAL_ADCEx_MultiModeConfigChannel().

                       *** Execution of ADC conversions ***
  ==============================================================================
  [..]
  (#) ADC driver can be used among three modes: polling, interruption,
      transfer by DMA.

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Start the ADC peripheral using HAL_ADC_Start()
       (+) Wait for end of conversion using HAL_ADC_PollForConversion(), at this stage
           user can specify the value of timeout according to his end application
       (+) To read the ADC converted values, use the HAL_ADC_GetValue() function.
       (+) Stop the ADC peripheral using HAL_ADC_Stop()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Start the ADC peripheral using HAL_ADC_Start_IT()
       (+) Use HAL_ADC_IRQHandler() called under ADC_IRQHandler() Interrupt subroutine
       (+) At ADC end of conversion HAL_ADC_ConvCpltCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ConvCpltCallback
       (+) In case of ADC Error, HAL_ADC_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ErrorCallback
       (+) Stop the ADC peripheral using HAL_ADC_Stop_IT()

     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Start the ADC peripheral using HAL_ADC_Start_DMA(), at this stage the user specify the length
           of data to be transferred at each end of conversion
       (+) At The end of data transfer by HAL_ADC_ConvCpltCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ConvCpltCallback
       (+) In case of transfer Error, HAL_ADC_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ErrorCallback
       (+) Stop the ADC peripheral using HAL_ADC_Stop_DMA()

     *** ADC HAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in ADC HAL driver.

      (+) __HAL_ADC_ENABLE : Enable the ADC peripheral
      (+) __HAL_ADC_DISABLE : Disable the ADC peripheral
      (+) __HAL_ADC_ENABLE_IT: Enable the ADC end of conversion interrupt
      (+) __HAL_ADC_DISABLE_IT: Disable the ADC end of conversion interrupt
      (+) __HAL_ADC_GET_IT_SOURCE: Check if the specified ADC interrupt source is enabled or disabled
      (+) __HAL_ADC_CLEAR_FLAG: Clear the ADC's pending flags
      (+) __HAL_ADC_GET_FLAG: Get the selected ADC's flag status
      (+) ADC_GET_RESOLUTION: Return resolution bits in CR1 register

     *** Callback functions ***
     ==============================
     [..]
      (@) Callback functions must be implemented in user program:
      (+@) HAL_ADC_ErrorCallback()
      (+@) HAL_ADC_LevelOutOfWindowCallback() (callback of analog watchdog)
      (+@) HAL_ADC_ConvCpltCallback()
      (+@) HAL_ADC_ConvHalfCpltCallback

       (@) You can refer to the ADC HAL driver header file for more useful macros

                      *** Deinitialization of ADC ***
  ==============================================================================
  [..]
  (#) Disable the ADC interface
     (++) ADC clock can be hard reset and disabled at RCC top level.
     (++) Hard reset of ADC peripherals
          using macro __HAL_RCC_ADC_FORCE_RESET(), __HAL_RCC_ADC_RELEASE_RESET().
     (++) ADC clock disable using the equivalent macro/functions as configuration step.
               (+++) Example:
                   Into HAL_ADC_MspDeInit() (recommended code location) or with
                   other device clock parameters configuration:
               (+++) HAL_RCC_GetOscConfig(&RCC_OscInitStructure);
               (+++) RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI;
               (+++) RCC_OscInitStructure.HSIState = RCC_HSI_OFF; (if not used for system clock)
               (+++) HAL_RCC_OscConfig(&RCC_OscInitStructure);

  (#) ADC pins configuration
     (++) Disable the clock for the ADC GPIOs using macro __HAL_RCC_GPIOx_CLK_DISABLE()

  (#) Optionally, in case of usage of ADC with interruptions:
     (++) Disable the NVIC for ADC using function HAL_NVIC_DisableIRQ(ADCx_IRQn)

  (#) Optionally, in case of usage of DMA:
        (++) Deinitialize the DMA using function HAL_DMA_DeInit().
        (++) Disable the NVIC for DMA using function HAL_NVIC_DisableIRQ(DMAx_Channelx_IRQn)

                      *** Callback registration ***
  ==============================================================================
    [..]

     The compilation flag USE_HAL_ADC_REGISTER_CALLBACKS, when set to 1,
     allows the user to configure dynamically the driver callbacks.
     Use Functions @ref HAL_ADC_RegisterCallback()
     to register an interrupt callback.
    [..]

     Function @ref HAL_ADC_RegisterCallback() allows to register following callbacks:
       (+) ConvCpltCallback               : ADC conversion complete callback
       (+) ConvHalfCpltCallback           : ADC conversion DMA half-transfer callback
       (+) LevelOutOfWindowCallback       : ADC analog watchdog 1 callback
       (+) ErrorCallback                  : ADC error callback
       (+) InjectedConvCpltCallback       : ADC group injected conversion complete callback
       (+) InjectedQueueOverflowCallback  : ADC group injected context queue overflow callback
       (+) LevelOutOfWindow2Callback      : ADC analog watchdog 2 callback
       (+) LevelOutOfWindow3Callback      : ADC analog watchdog 3 callback
       (+) EndOfSamplingCallback          : ADC end of sampling callback
       (+) MspInitCallback                : ADC Msp Init callback
       (+) MspDeInitCallback              : ADC Msp DeInit callback
     This function takes as parameters the HAL peripheral handle, the Callback ID
     and a pointer to the user callback function.
    [..]

     Use function @ref HAL_ADC_UnRegisterCallback to reset a callback to the default
     weak function.
    [..]

     @ref HAL_ADC_UnRegisterCallback takes as parameters the HAL peripheral handle,
     and the Callback ID.
     This function allows to reset following callbacks:
       (+) ConvCpltCallback               : ADC conversion complete callback
       (+) ConvHalfCpltCallback           : ADC conversion DMA half-transfer callback
       (+) LevelOutOfWindowCallback       : ADC analog watchdog 1 callback
       (+) ErrorCallback                  : ADC error callback
       (+) InjectedConvCpltCallback       : ADC group injected conversion complete callback
       (+) InjectedQueueOverflowCallback  : ADC group injected context queue overflow callback
       (+) LevelOutOfWindow2Callback      : ADC analog watchdog 2 callback
       (+) LevelOutOfWindow3Callback      : ADC analog watchdog 3 callback
       (+) EndOfSamplingCallback          : ADC end of sampling callback
       (+) MspInitCallback                : ADC Msp Init callback
       (+) MspDeInitCallback              : ADC Msp DeInit callback
     [..]

     By default, after the @ref HAL_ADC_Init() and when the state is @ref HAL_ADC_STATE_RESET
     all callbacks are set to the corresponding weak functions:
     examples @ref HAL_ADC_ConvCpltCallback(), @ref HAL_ADC_ErrorCallback().
     Exception done for MspInit and MspDeInit functions that are
     reset to the legacy weak functions in the @ref HAL_ADC_Init()/ @ref HAL_ADC_DeInit() only when
     these callbacks are null (not registered beforehand).
    [..]

     If MspInit or MspDeInit are not null, the @ref HAL_ADC_Init()/ @ref HAL_ADC_DeInit()
     keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.
     [..]

     Callbacks can be registered/unregistered in @ref HAL_ADC_STATE_READY state only.
     Exception done MspInit/MspDeInit functions that can be registered/unregistered
     in @ref HAL_ADC_STATE_READY or @ref HAL_ADC_STATE_RESET state,
     thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
    [..]

     Then, the user first registers the MspInit/MspDeInit user callbacks
     using @ref HAL_ADC_RegisterCallback() before calling @ref HAL_ADC_DeInit()
     or @ref HAL_ADC_Init() function.
     [..]

     When the compilation flag USE_HAL_ADC_REGISTER_CALLBACKS is set to 0 or
     not defined, the callback registration feature is not available and all callbacks
     are set to the corresponding weak functions.

    @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"

/* Private function prototypes -----------------------------------------------*/
static void ADC_DMAConvCplt0 (DMA_HandleTypeDef *hdma);
static void ADC_DMAConvCplt1 (DMA_HandleTypeDef *hdma);
static void ADC_DMAError (DMA_HandleTypeDef *hdma);

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
__weak void HAL_ADC_DB_ConvCpltCallback(ADC_HandleTypeDef* hadc, int which)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  UNUSED(which);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
}


/**
  * @brief  Enables ADC DMA request after last transfer (Single-ADC mode) and enables ADC peripheral
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from ADC peripheral to memory.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Start_DB_DMA (ADC_HandleTypeDef* hadc, uint32_t* pData1, uint32_t* pData2, uint32_t Length)
{
  __IO uint32_t counter = 0;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
  assert_param(IS_ADC_EXT_TRIG_EDGE(hadc->Init.ExternalTrigConvEdge));

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Enable the ADC peripheral */
  /* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization */
  if((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
  {
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE(hadc);

    /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
    counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000));
    while(counter != 0)
    {
      counter--;
    }
  }

  /* Start conversion if ADC is effectively enabled */
  if(HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON))
  {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
                      HAL_ADC_STATE_REG_BUSY);

    /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
    if (READ_BIT(hadc->Instance->CR1, ADC_CR1_JAUTO) != RESET)
    {
      ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);
    }

    /* State machine update: Check if an injected conversion is ongoing */
    if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
    {
      /* Reset ADC error code fields related to conversions on group regular */
      CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
    }
    else
    {
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE(hadc);
    }

    /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
    __HAL_UNLOCK(hadc);

    /* Set the DMA transfer complete callback */
    hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt0;
    hadc->DMA_Handle->XferM1CpltCallback = ADC_DMAConvCplt1;

    /* Set the DMA error callback */
    hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


    /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
    /* start (in case of SW start):                                           */

    /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);

    /* Enable ADC overrun interrupt */
    __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

    /* Enable ADC DMA mode */
    hadc->Instance->CR2 |= ADC_CR2_DMA;

    /* Start the DMA channel */
    HAL_DMAEx_MultiBufferStart_IT (hadc->DMA_Handle, (uint32_t) &hadc->Instance->DR, (uint32_t) pData1, (uint32_t) pData2, Length);

    /* Check if Multimode enabled */
    if(HAL_IS_BIT_CLR(ADC->CCR, ADC_CCR_MULTI))
    {
      /* if no external trigger present enable software conversion of regular channels */
      if((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET)
      {
        /* Enable the selected ADC software conversion for regular group */
        hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
    }
    else
    {
      /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
      if((hadc->Instance == ADC1) && ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
      {
        /* Enable the selected ADC software conversion for regular group */
          hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
      /* if dual mode is selected, ADC3 works independently. */
      /* check if the mode selected is not triple */
      if( HAL_IS_BIT_CLR(ADC->CCR, ADC_CCR_MULTI_4) )
      {
        /* if instance of handle correspond to ADC3 and  no external trigger present enable software conversion of regular channels */
        if((hadc->Instance == ADC3) && ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
        {
          /* Enable the selected ADC software conversion for regular group */
          hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        }
      }
    }
  }

  /* Return function status */
  return HAL_OK;
}



/**
  * @brief  DMA transfer complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void ADC_DMAConvCplt (DMA_HandleTypeDef *hdma, int which)
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Update state machine on conversion status if not in error state */
  if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))
  {
    /* Update ADC state machine */
    SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

    /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On STM32F7, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
    if(ADC_IS_SOFTWARE_START_REGULAR(hadc)                   &&
       (hadc->Init.ContinuousConvMode == DISABLE)            &&
       (HAL_IS_BIT_CLR(hadc->Instance->SQR1, ADC_SQR1_L) ||
        HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_EOCS)  )   )
    {
      /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
      __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

      /* Set ADC state */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

      if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      {
        SET_BIT(hadc->State, HAL_ADC_STATE_READY);
      }
    }

    /* Conversion complete callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
    hadc->ConvCpltCallback(hadc);
#else
    HAL_ADC_DB_ConvCpltCallback(hadc, which);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
  }
  else /* DMA and-or internal error occurred */
  {
    if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) != 0UL)
    {
      /* Call HAL ADC Error Callback function */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
      hadc->ErrorCallback(hadc);
#else
      HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
    }
    else
    {
      /* Call DMA error callback */
      hadc->DMA_Handle->XferErrorCallback(hdma);
    }
  }
}

static void ADC_DMAConvCplt0 (DMA_HandleTypeDef *hdma)
{
	ADC_DMAConvCplt (hdma, 0);
}

static void ADC_DMAConvCplt1 (DMA_HandleTypeDef *hdma)
{
	ADC_DMAConvCplt (hdma, 1);
}


/**
  * @brief  DMA error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void ADC_DMAError(DMA_HandleTypeDef *hdma)
{
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hadc->State= HAL_ADC_STATE_ERROR_DMA;
  /* Set ADC error code to DMA error */
  hadc->ErrorCode |= HAL_ADC_ERROR_DMA;
  /* Error callback */
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
  hadc->ErrorCallback(hadc);
#else
  HAL_ADC_ErrorCallback(hadc);
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
}
