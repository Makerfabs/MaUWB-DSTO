#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "esp_lcd_panel_vendor.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "lwip/err.h"
#include "lwip/sys.h"



static const char *TAG = "uwb_main";

#define I2C_BUS_PORT    0
#define CONFIG_EXAMPLE_SSD1306_HEIGHT 64

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           39
#define EXAMPLE_PIN_NUM_SCL           38
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              CONFIG_EXAMPLE_SSD1306_HEIGHT

#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define UWB_UART_NUM    UART_NUM_2
#define PC_UART_NUM     UART_NUM_0

#define UWB_RX_PIN      15
#define UWB_TX_PIN      16

// Max Frame is 239 bytes. 1024 is enough for ~4 frames.
#define BUF_SIZE        1024 

#define DSTO_HEAD      0x2A
#define DSTO_FOOT      0x23
#define CMD_TYPE_RANGE 0x71
#define CMD_TYPE_CFG   0x02 


#define MAX_PAYLOAD_LEN 240 

// --- LVGL UI objects ---
static lv_obj_t *label_list;

typedef union {
    struct __attribute__((packed)) { 
        uint64_t  dev_mode  :2;
        uint64_t  dev_id    :32;
        uint64_t  dev_type  :3;
        uint64_t  done      :1;
        uint64_t  inNet     :1;
        uint64_t  reserved  :25;
    } bit;
    uint64_t value;
} dev_para_u;


static uint8_t get_Xor_CRC(uint8_t *bytes, int offset, int len) {
    uint8_t xor_crc = 0;
    for (int i = 0; i < len; i++) {
        xor_crc ^= bytes[offset + i];
    }
    return xor_crc;
}

static void uwb_process_packet(uint8_t *data, uint16_t payload_len) {
    uint8_t pkt_crc = data[3 + payload_len];
    uint8_t calc_crc = get_Xor_CRC(data, 3, payload_len);

    if (pkt_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC Error: Calc 0x%02X != Pkt 0x%02X", calc_crc, pkt_crc);
        return;
    }

    uint8_t cmd_type = data[19];

    if (cmd_type == CMD_TYPE_RANGE) {
        // data[36] is online_dev_num
        uint8_t online_dev_num = data[36];
        // ESP_LOGI(TAG, "Range Frame: Online devices: %d", online_dev_num);

        char lvgl_str[256] = {0}; 
        int lvgl_str_len = 0;
        int display_count = 0;

        for (int i = 0; i < online_dev_num ; i++) {
            dev_para_u dev_para;
            int base_idx = 37 + i * 10;

            if (base_idx + 10 > 3 + payload_len) break;

            memcpy(&dev_para.value, &data[base_idx], 8);

            if (dev_para.bit.dev_id == 0) continue; 


            uint16_t dis;
            memcpy(&dis, &data[base_idx + 8], 2);

            if (display_count < 6) {
                int added = snprintf(lvgl_str + lvgl_str_len, sizeof(lvgl_str) - lvgl_str_len, 
                                     "ID%d:%d cm\n", 
                                     dev_para.bit.dev_id, dis);
                if (added > 0) lvgl_str_len += added;
                display_count++;
            }
            
            // ESP_LOGI(TAG, "  DevID: %d, Distance: %d cm", (int)dev_para.bit.dev_id, dis);
        }

        if (lvgl_port_lock(0)) {
            if (online_dev_num == 0) {
                lv_label_set_text(label_list, "No Devices");
            } else {
                lv_label_set_text(label_list, lvgl_str);
            }
            lvgl_port_unlock();
        }
        
    } else if (cmd_type == CMD_TYPE_CFG) {
        ESP_LOGI(TAG, "Received configuration packet");
    } else {
        ESP_LOGD(TAG, "Other command type: 0x%02X", cmd_type);
    }
}

static void pc_rx_task(void *arg) {
    uint8_t *data = (uint8_t *) malloc(1024);
    while (1) {
        int len = uart_read_bytes(PC_UART_NUM, data, 1024 - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            esp_log_buffer_hex(TAG, data, len);
            uart_write_bytes(UWB_UART_NUM, data, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    free(data);
}

static void uwb_rx_task(void *arg) {
    static uint8_t rx_buffer[BUF_SIZE];
    static int current_len = 0;

    while (1) {
        int free_space = BUF_SIZE - current_len;
        if (free_space <= 0) {
            ESP_LOGE(TAG, "Buffer overflow, clearing");
            current_len = 0;
            free_space = BUF_SIZE;
        }

        int len = uart_read_bytes(UWB_UART_NUM, rx_buffer + current_len, free_space, pdMS_TO_TICKS(20));

        if (len > 0) {
            uart_write_bytes(PC_UART_NUM, rx_buffer + current_len, len);
            
            current_len += len;

            int offset = 0;
            while (offset < current_len) {
                int remain = current_len - offset;

                if (rx_buffer[offset] != DSTO_HEAD) {
                    offset++;
                    continue; 
                }

                if (remain < 3) {
                    break; 
                }

                uint16_t payload_len = rx_buffer[offset + 1] | (rx_buffer[offset + 2] << 8);
                
                if (payload_len > MAX_PAYLOAD_LEN) {
                    ESP_LOGW(TAG, "Invalid payload len %d, skipping fake header", payload_len);
                    offset++;
                    continue;
                }

                int frame_total_len = 1 + 2 + payload_len + 1 + 1;

                if (remain < frame_total_len) {
                    break;
                }

                if (rx_buffer[offset + frame_total_len - 1] != DSTO_FOOT) {
                    ESP_LOGW(TAG, "Header found but Footer mismatch, skipping byte");
                    offset++;
                    continue;
                }

                uwb_process_packet(&rx_buffer[offset], payload_len);

                offset += frame_total_len;
            }

            if (offset > 0) {
                int remaining_data = current_len - offset;
                if (remaining_data > 0) {
                    memmove(rx_buffer, rx_buffer + offset, remaining_data);
                }
                current_len = remaining_data;
            }
        }
        
        if (len == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void app_main(void)
{
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 1;
    port_cfg.task_stack = 6144;
#if CONFIG_SOC_CPU_CORES_NUM > 1
    port_cfg.task_affinity = 1;
#endif
    lvgl_port_init(&port_cfg);

    ESP_LOGI(TAG, "Adding OLED display");

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = false,
        .trans_size = 0,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = 1,
            .buff_spiram = 0,
            .sw_rotate = 0,
            .full_refresh = 0,
            .direct_mode = 0,
        },
    };
    lv_display_t *display_ = lvgl_port_add_disp(&disp_cfg);

    if (lvgl_port_lock(0)) {
        label_list = lv_label_create(lv_screen_active());
        lv_label_set_long_mode(label_list, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(label_list, 128);
        lv_obj_align(label_list, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_text_font(label_list, &lv_font_montserrat_10, 0);
        lv_label_set_text(label_list, "Waiting for UWB...");
        lvgl_port_unlock();
    }

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PC_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PC_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PC_UART_NUM, 43, 44, -1, -1));

    uart_config_t uwb_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UWB_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UWB_UART_NUM, &uwb_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UWB_UART_NUM, UWB_TX_PIN, UWB_RX_PIN, -1, -1));

    xTaskCreate(pc_rx_task, "pc_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(uwb_rx_task, "uwb_rx_task", 4096, NULL, 12, NULL); 
}