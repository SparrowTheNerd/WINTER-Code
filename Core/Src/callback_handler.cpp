#include "stm32h7xx_hal.h"
#include "spi.h"
#include "sdDMA.h"

// Handles callbacks from GPIO interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == SD_MISO_Pin) {
        SD_Ready_Callback();
    }
}

// Handles callbacks from SPI TxRx completion
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi->Instance == SD_SPI_HANDLE.Instance) {
		SD_Busy_Wait(); // when one block completes, begin waiting for card to be ready

	}
}
