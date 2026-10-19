#include "stm32h7xx_hal.h"
#include "spi.h"
#include "sdDMA.h"

//// Handles callbacks from GPIO interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == SD_MISO_Pin) {
        // Do not abort/restart SPI DMA from an EXTI handler.  HAL_SPI_Abort()
        // can wait for the SPI hardware and starve the polling I2C transfers.
        // logMachine() performs that work in normal thread context instead.
        SD_Ready_Notify();
    }
}

// Handles callbacks from SPI TxRx completion
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi->Instance == SD_SPI_HANDLE.Instance) {
		SD_Busy_Wait(); // when one block completes, begin waiting for card to be ready be ready

	}
}