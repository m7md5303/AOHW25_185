#include <stdio.h>
#include <stdint.h>
#include "img2.h"  // image in hex
#include "xil_printf.h"
#include "xdebug.h"
#include "sleep.h"
#include "xstatus.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xvidc.h"
#include "xparameters.h"
#include "xaxidma.h"

// Define important parameters
#define IMG_LENGTH              173056
#define POLL_TIMEOUT_COUNTER    1000000U
#define DDR_BASEADDR            XPAR_DDR_MEM_BASEADDR
#define DMA_DEV_ID              XPAR_AXIDMA_0_DEVICE_ID
#define MEM_BASE_ADDR           (DDR_BASEADDR + 0x1000000)
#define TX_BUFFER_BASE          (MEM_BASE_ADDR + 0x00100000)

// DMA instance
XAxiDma AxiDma;
XAxiDma_Config *DMA_Cfg;

// Allocate aligned buffer for image
volatile uint32_t TxBuffer[IMG_LENGTH] __attribute__((aligned(32)));

int main(void) {
    int status;
    xil_printf("Welcome from the main function\n");

    // Initialize DMA
    DMA_Cfg = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!DMA_Cfg) {
        xil_printf("DMA config lookup failed\n");
        return XST_FAILURE;
    }

    status = XAxiDma_CfgInitialize(&AxiDma, DMA_Cfg);
    if (status != XST_SUCCESS) {
        xil_printf("DMA init failed - %x\n", status);
        return XST_FAILURE;
    } else {
        xil_printf("DMA init success - %x\n", status);
    }

    // Copy image data into buffer
    for (int i = 0; i < IMG_LENGTH; i++) {
        TxBuffer[i] = hex_values[i];
    }

    // Flush cache to ensure buffer is updated in memory
    Xil_DCacheFlushRange((UINTPTR)TxBuffer, IMG_LENGTH * sizeof(uint32_t));
while(1){
    // Start DMA transfer to device
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)TxBuffer, IMG_LENGTH * sizeof(uint32_t), XAXIDMA_DMA_TO_DEVICE);
    if (status != XST_SUCCESS) {
        xil_printf("DMA transfer failed - %x\n", status);
        return XST_FAILURE;
    }


    // Wait for DMA transfer to complete
    int TimeOut = POLL_TIMEOUT_COUNTER;
    while (TimeOut) {
        if (!XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE)) {
            xil_printf("DMA transfer completed successfully\n");
            break;
        }
        TimeOut--;
        usleep(1U);
    }

    if (TimeOut == 0) {
        xil_printf("DMA transfer timed out\n");
        return XST_FAILURE;
    }

    xil_printf("Image sent via DMA successfully\n");
}
    // Idle loop or additional logic here
    while (1) {
        // You can add GPIO status checks or DMA status here
    }

    return 0;
}
