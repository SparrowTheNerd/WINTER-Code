#include "sdDMA.h"

packet dataPacket;
SD_State_t SD_State = SD_IDLE;

FATFS SDFatFS;              // The FATFS object
FILINFO fno;

char FILE_NAME[] = "FLIGHTDATA00.bin";
constexpr size_t FILE_SIZE = 64 * 1024 * 1024; // 64 MB
constexpr size_t BLOCK_SIZE = 512;
constexpr size_t numPerBlock = BLOCK_SIZE / sizeof(dataPacket);
constexpr size_t BLOCK_FRAME_SIZE = BLOCK_SIZE + EXTRA_BYTES; // 512 bytes + 2 bytes for CRC
constexpr uint32_t dmaBlocks = 31;
constexpr size_t BUFFER_SIZE = BLOCK_FRAME_SIZE * dmaBlocks; // blocks plus CRC
constexpr size_t NUM_BLOCKS = FILE_SIZE / BLOCK_SIZE;

static uint8_t bufA[16384] __attribute__((section(".nocachetx"))); // Buffer A for write operations
static uint8_t bufB[16384] __attribute__((section(".nocachetx"))); // Buffer B for write operations
static uint8_t rxBuffer[1024] __attribute__((section(".nocacherx"))); // Buffer for read operations

static uint8_t* logBuf = bufA;
static uint8_t* writeBuf = NULL;
static uint8_t* pendingBuf = NULL;

volatile static uint8_t sd_clear = 1; // Flag to indicate that the SD card is ready for the next transfer

DWORD lba_start;

FIL file;
FRESULT res;
DSTATUS dres;

void sdInit() {
    if (f_mount(&SDFatFS, "", 1) != FR_OK) {
            SerialPrintln((uint8_t*)"Failed to mount SD card.\r\n");
            while(1);
        }
    else SerialPrintln((uint8_t*)"SD card mounted successfully.\r\n");


    for (uint8_t i = 0; i < 100; i++) {     // create a file with a unique incremented name
        FILE_NAME[10] = i/10 + '0';
        FILE_NAME[11] = i%10 + '0';
        if (f_stat(FILE_NAME, &fno) == FR_OK) continue; // if file already exists, increment to the next one
        
        if (f_open(&file, FILE_NAME, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
            SerialPrintln((uint8_t*)"Failed to create file.");
            while(1);
        }
        else {
            SerialPrintln((uint8_t*)"File created successfully.");
            break;
        }
    }

    f_expand(&file, FILE_SIZE,1); // Expand file to desired size
    f_lseek(&file, f_size(&file));
    // DWORD lba_start = 0;
    lba_start = ((file.obj.sclust - 2) * SDFatFS.csize) + SDFatFS.database;   //calculate the starting LBA
    f_close(&file);
}
   

void logMachine() {  // SD card state machine for async writing
    static uint32_t idx = 0;
    static DWORD sector = lba_start + idx*dmaBlocks;
    const uint8_t cmd25[5] = {0x40 | 25, (uint8_t)(sector >> 24), (uint8_t)(sector >> 16), (uint8_t)(sector >> 8), (uint8_t)sector};
    logData();
    
    switch (SD_State) {
        case SD_IDLE:
            if (writeBuf != NULL) {
                SD_State = SD_OPEN;
            }
            break;
        case SD_OPEN:
            f_open(&file, FILE_NAME, FA_WRITE | FA_OPEN_EXISTING);
            HAL_SPI_Transmit(&SD_SPI_HANDLE, cmd25, 5, HAL_MAX_DELAY);
            HAL_SPI_TransmitReceive_DMA(&SD_SPI_HANDLE, writeBuf, NULL, BLOCK_FRAME_SIZE);
            sd_clear = 0;
            SD_State = SD_STREAMING;
            break;
        case SD_STREAMING:
            if(sd_clear) {
                SD_State = SD_CLOSE;
                idx++;
                SerialPrintln((uint8_t*)"Block written to SD card.");
            }
            break;
        case SD_CLOSE:
            f_close(&file);
            writeBuf = NULL;
            if(pendingBuf != NULL) {
                writeBuf = pendingBuf;
                pendingBuf = NULL;
                SD_State = SD_OPEN;
            }
            else { SD_State = SD_IDLE; }
    }
}


void logData() {
    static uint32_t idx = 0;
    static uint8_t ofst = 0;

    if (idx < numPerBlock * dmaBlocks) {
        ofst += (idx % 2 == 0 && idx !=0) ? 1 : 0; // ofst should be 0, 0, 1, 1, 2, 2, etc so increment every two packets but not on the first packet

        if(idx == 0) {
            logBuf[0] = 0xFC; // start byte for first block
        }

        memcpy(logBuf + idx*sizeof(dataPacket)+ofst*EXTRA_BYTES + 1, &dataPacket, sizeof(dataPacket)); // copy data packet to buffer

        if(idx % 2 == 1) {  // if on the boundary of data and crc, add crc to buffer
            for(int i=0; i<EXTRA_BYTES-1; i++) {
                logBuf[idx*sizeof(dataPacket)+ofst*EXTRA_BYTES+i+1] = 0xFF; // add CRC bytes and additional clocks
            }
            logBuf[idx*sizeof(dataPacket)+ofst*EXTRA_BYTES+EXTRA_BYTES] = 0xFC; // add start byte for next block
        }

        idx++;
        return;
    }
    else { ofst = 0; }

    if (writeBuf != NULL) {
        pendingBuf = logBuf;
        logBuf = (logBuf == bufA) ? bufB : bufA; // swap logging buffer if current buffer is full
        idx = 0;
    }

    else {
        writeBuf = logBuf;
        logBuf = (logBuf == bufA) ? bufB : bufA; // swap logging buffer if current buffer is full
        idx = 0;
    }
}

extern DMA_HandleTypeDef hdma_spi1_tx;

void SD_Busy_Wait() {
	const uint8_t busyByte = 0xFF;

	((DMA_Stream_TypeDef *)hdma_spi1_tx.Instance)->CR &= ~DMA_SxCR_MINC; // Disable memory increment mode

	__HAL_GPIO_EXTI_CLEAR_IT(SD_MISO_Pin);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);	// Turn MISO into an interrupt - when SD card is ready it will pull MISO high

    SD_SPI_HANDLE.Instance->CFG1 = (SD_SPI_HANDLE.Instance->CFG1 & ~SPI_CFG1_MBR) | SPI_BAUDRATEPRESCALER_256; // Slow down SPI clock

	HAL_SPI_Transmit_DMA(&SD_SPI_HANDLE, (uint8_t*)&busyByte, 65535); // Start a DMA transfer to read the busy state of the SD card

}

extern SPI_HandleTypeDef SD_SPI_HANDLE;

void SD_Ready_Callback() {
    const uint8_t end = 0xFD; // Stop token for multiple block write
	static uint8_t idx = 1;

	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	HAL_SPI_Abort(&SD_SPI_HANDLE); // Stop the DMA transfer
    SD_SPI_HANDLE.Instance->CFG1 = (SD_SPI_HANDLE.Instance->CFG1 & ~SPI_CFG1_MBR) | SPI_BAUDRATEPRESCALER_32;
	((DMA_Stream_TypeDef *)hdma_spi1_tx.Instance)->CR |= DMA_SxCR_MINC; // Re-enable memory increment mode

	if(idx < dmaBlocks) {
		HAL_SPI_TransmitReceive_DMA(&SD_SPI_HANDLE, writeBuf + idx*(512+EXTRA_BYTES), NULL, (512+EXTRA_BYTES));
		idx++;
	}
	else {
		idx = 1; // Reset for next transfer
		HAL_SPI_Transmit(&SD_SPI_HANDLE, &end, 1, HAL_MAX_DELAY); // Send stop token
        sd_clear = 1;
	}

}
