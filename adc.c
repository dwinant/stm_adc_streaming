/*
 * adc.c
 *
 *  Created on: Dec 30, 2020
 *      Author: david.winant
 */
#include "main.h"
#include "output.h"

#define BUF0_SIZE		1000

enum { FALSE, TRUE };

extern ADC_HandleTypeDef hadc3;
extern HAL_StatusTypeDef HAL_ADC_Start_DB_DMA (ADC_HandleTypeDef* hadc, uint32_t* pData1, uint32_t* pData2, uint32_t Length);

uint16_t mem[2][BUF0_SIZE] = {{0}, {0}};
uint32_t conversion_count[2] = {0};

int buffer_to_process = 0;
int go_ahead_process_buffer = FALSE;


static inline flag_up (void)
{
	HAL_GPIO_WritePin (ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin, 1);
}

static inline flag_down (void)
{
	HAL_GPIO_WritePin (ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin, 0);
}

static inline flag_set (int v)
{
	HAL_GPIO_WritePin (ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin, v);
}



void process_buffer (int which)
{
	static int processed = 0;
	uint32_t sum = 0;

	for (int i = 0; i < BUF0_SIZE; i++)
		sum += mem[which][i];

	if (++processed % 7 == 0) {
		flag_up();
		output (" buf %d sum %08x avg %x\r\n", which, sum, sum / BUF0_SIZE);
		flag_down();
	}
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_DB_ConvCpltCallback(ADC_HandleTypeDef* hadc, int which)
{
  buffer_to_process = which;
  go_ahead_process_buffer = TRUE;

  conversion_count[which]++;
}



void adc_process (void)
{
  int status = HAL_ADC_Start_DB_DMA (&hadc3, (uint32_t*) mem[0], (uint32_t*) mem[1], BUF0_SIZE);

  if (status != HAL_OK) {
	  output ("Could not start ADC DMA capture\r\n");
	  return;
  }

  output ("A/D started\r\n");

  while (1) {
	  if (go_ahead_process_buffer) {
		  int which = buffer_to_process;
		  go_ahead_process_buffer = FALSE;

		  process_buffer (which);
	  }
  }
}
