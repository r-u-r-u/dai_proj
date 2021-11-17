#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp32_spi_impl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "spi_api.hpp"
#include <cmath>

#define ECHO_TEST_TXD (17)//17
#define ECHO_TEST_RXD (16)//16
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)//
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)//
#define ECHO_UART_PORT_NUM      (2)//2
#define ECHO_UART_BAUD_RATE     (115200)//115200
#define ECHO_TASK_STACK_SIZE    (2048)//2048
#define BUF_SIZE (1024)


extern "C" {
   void app_main();
}

void run_demo(){
    uint8_t req_success = 0;

    dai::SpiApi mySpiApi;
    mySpiApi.set_send_spi_impl(&esp32_send_spi);
    mySpiApi.set_recv_spi_impl(&esp32_recv_spi);

    bool receivedAnyMessage = false;
    while(1) {
        dai::Message servoDataMsg;
        if(mySpiApi.req_message(&servoDataMsg, "servo")){
            uart_write_bytes(ECHO_UART_PORT_NUM, servoDataMsg.raw_data.data, servoDataMsg.raw_data.size);
            printf("received! m:%s d:%d t:%d\n",servoDataMsg.raw_data.data,servoDataMsg.raw_data.size,servoDataMsg.type==dai::DatatypeEnum::Buffer);
            mySpiApi.free_message(&servoDataMsg);
            mySpiApi.spi_pop_message("servo");
            receivedAnyMessage = true;
        }

        if(!receivedAnyMessage){
            usleep(1000);
        }
    }
}

//Main application
void app_main()
{
    init_esp32_spi();

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // init spi for the esp32
    run_demo();

    //Never reached.
    deinit_esp32_spi();
}
