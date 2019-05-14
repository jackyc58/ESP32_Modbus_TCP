/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"
#include "mbcrc.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "driver/gpio.h"

#define PORT    502

// Read packet timeout
#define SLAVE_RESPONSE_WAIT_TIME        500 // ms
#define CHECK_RESPONSE_INTERVAL         20 // ms

#define MB_UART_TXD     (17)
#define MB_UART_RXD     (5)
#define MB_UART_RTS     (16)
#define MB_PORT_NUM     (2)           // Number of UART port used for Modbus connection
#define MB_DEV_SPEED    (57600)       // The communication speed of the UART
#define BUF_SIZE        (128)


#define RX_SIZE     32
#define TX_SIZE     64
#define MBAP_LEN    6
#define CRC_LEN     2

static const char *TAG = "example";
const int uart_num = MB_PORT_NUM;

void printfPacket(char *fieldName, char *data, int dLen)
{
    int i;
    
    printf("%s: '", fieldName);
    for (i=0; i < dLen; i++)
        printf("%02X ", data[i]);
    printf("'\n");    
}

void init_uart()
{    
    uart_config_t uart_config = {
        .baud_rate = MB_DEV_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Configure UART parameters
    uart_param_config(uart_num, &uart_config);
    
    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    // Set UART1 pins
    uart_set_pin(uart_num, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS, UART_PIN_NO_CHANGE);

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE+1, 0, 0, NULL, 0);

    // Set RS485 half duplex mode
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);    
}


static void tcp_server_task(void *pvParameters)
{
    char rx_buf[RX_SIZE], tx_buf[TX_SIZE];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    int err;
    int listen_sock;

#ifdef CONFIG_EXAMPLE_IPV4
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
    struct sockaddr_in6 dest_addr;
    bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

    listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        //break;
    }
    ESP_LOGI(TAG, "Socket created");

    err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        //break;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    while (1) {
        uint addr_len;
        int sock;
        
        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");

        while (1) {
            int rx_len_s, rx_len_u, aduSize, loopTime;
            char mbapHeader[7], *aduData;
            uint16_t crc;
            uint8_t *rxSlaveData;
            
            rx_len_s = recv(sock, rx_buf, RX_SIZE - 1, 0);
            
            // Error occurred during receiving
            if (rx_len_s < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (rx_len_s == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
                
                // get MBAP Header
                memcpy(mbapHeader, rx_buf, MBAP_LEN);
                mbapHeader[MBAP_LEN] = 0;
                printfPacket("mbapHeader", mbapHeader, MBAP_LEN);
                
                // get ADU packet
                aduSize = rx_len_s - MBAP_LEN + CRC_LEN;
                //printf("rxBufSize=%d\n", rxBufSize);
                aduData = malloc(aduSize + 1);
                memcpy(aduData, rx_buf + MBAP_LEN, aduSize - CRC_LEN);
                
                printfPacket("ADU", aduData, aduSize - CRC_LEN); // print ADU               
                
                // CRC
                crc = usMBCRC16((uint8_t *) aduData, aduSize - CRC_LEN );
                aduData[aduSize-2] = (uint8_t)( crc & 0xFF );
                aduData[aduSize-1] = (uint8_t)( crc >> 8 );

                //ESP_LOGI(TAG, "UART start recieve loop.\r\n");
                uart_write_bytes(uart_num, aduData, aduSize);   // send data to Slave by uart
                //vTaskDelay(100 / portTICK_RATE_MS);

                //Read data from UART
                rxSlaveData = (uint8_t*) malloc(TX_SIZE);    // Allocate buffers for UART
                loopTime=0;
                
                while (loopTime < SLAVE_RESPONSE_WAIT_TIME) {
                    // Receive slave data
                    rx_len_u = uart_read_bytes(uart_num, rxSlaveData, TX_SIZE, (CHECK_RESPONSE_INTERVAL / portTICK_RATE_MS));                                                      
      
                    //Write data back to UART
                    if (rx_len_u >= 6) {
                        int sLoc = 0;
                        printf("rx_len_u=%d\n", rx_len_u);
                        
                        
                        if (rxSlaveData[0] == 0)
                            sLoc = 1;
                        
                        printfPacket("RS485 receive", (char *)rxSlaveData, rx_len_u);
                    
                        memcpy(tx_buf, mbapHeader, MBAP_LEN);
                        memcpy(tx_buf+MBAP_LEN, rxSlaveData+sLoc, rx_len_u-sLoc-CRC_LEN);
                        
                        // TCP respond
                        printfPacket("TCP send", tx_buf, MBAP_LEN+(rx_len_u-sLoc-CRC_LEN));                        
                        err = send(sock, tx_buf, MBAP_LEN+(rx_len_u-sLoc-CRC_LEN), 0);

                        if (err < 0) 
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        
                        break;                        
                    }  

                    loopTime = loopTime + CHECK_RESPONSE_INTERVAL;
                }
                
                if (loopTime >= SLAVE_RESPONSE_WAIT_TIME)
                    printf("Slave(ID:%02X) not respondse!\n", aduData[0]);
              
                free(rxSlaveData);
                free(aduData);
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    
    init_uart();

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}
