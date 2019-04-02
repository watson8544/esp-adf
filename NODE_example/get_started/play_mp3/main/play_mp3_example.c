/* Play mp3 file by audio pipeline

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    For NODE Module supplied by M5Stack
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "audio_hal.h"
#include "filter_resample.h"
#include "zl38063.h"

#include "wm8978.h"

#include "esp32_digital_led_lib.h"

// #define CONFIG_ESP_M5CORE_NODE_BOARD 1

#define HIGH 1
#define LOW 0
#define OUTPUT GPIO_MODE_OUTPUT
#define INPUT GPIO_MODE_INPUT
#define nullptr  NULL

#define DATA_LENGTH                        128              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_EXAMPLE_MASTER_SCL_IO           22               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_MASTER_SDA_IO           21               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000  

#define ESP_SLAVE_ADDR                     0x5C             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#define GPIO_INPUT_IO_0     37
#define GPIO_INPUT_IO_1     38
#define GPIO_INPUT_IO_2     35
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))

static const char *TAG = "NODE";

strand_t STRANDS[] = { // Avoid using any of the strapping pins on the ESP32
  {.rmtChannel = 1, .gpioNum = 15, .ledType = LED_WS2812BW, .brightLimit = 100, .numPixels =  12,
   .pixels = nullptr, ._stateVars = nullptr},
};

int STRANDCNT = sizeof(STRANDS)/sizeof(STRANDS[0]);
// pixelColor_t colCur[64];
pixelColor_t colTarget[64];

/*
   To embed it in the app binary, the mp3 file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const uint8_t adf_music_mp3_start[] asm("_binary_adf_music_mp3_start");
extern const uint8_t adf_music_mp3_end[]   asm("_binary_adf_music_mp3_end");

int mp3_music_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    static int mp3_pos;
    int read_size = adf_music_mp3_end - adf_music_mp3_start - mp3_pos;
    if (read_size == 0) {
        return AEL_IO_DONE;
    } else if (len < read_size) {
        read_size = len;
    }
    memcpy(buf, adf_music_mp3_start + mp3_pos, read_size);
    mp3_pos += read_size;
    return read_size;
}

void gpioSetup(int gpioNum, int gpioMode, int gpioVal) {
    gpio_num_t gpioNumNative = (gpio_num_t)(gpioNum);
    gpio_mode_t gpioModeNative = (gpio_mode_t)(gpioMode);
    gpio_pad_select_gpio(gpioNumNative);
    gpio_set_direction(gpioNumNative, gpioModeNative);
    gpio_set_level(gpioNumNative, gpioVal);
};

void set_white_color(void)
{
    for (uint16_t i = 0; i < STRANDS[0].numPixels; i++) {
        colTarget[i] = pixelFromRGBW(20, 20, 20, 20);
    }
    for (uint16_t i = 0; i < STRANDS[0].numPixels; i++) {
		STRANDS[0].pixels[i] = colTarget[i];
	}
	digitalLeds_updatePixels(&STRANDS[0]);
}

void set_black_color(void)
{
    for (uint16_t i = 0; i < STRANDS[0].numPixels; i++) {
        colTarget[i] = pixelFromRGBW(0, 0, 0, 0);
    }
    for (uint16_t i = 0; i < STRANDS[0].numPixels; i++) {
		STRANDS[0].pixels[i] = colTarget[i];
	}
	digitalLeds_updatePixels(&STRANDS[0]);
}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_example_master_read_slave(i2c_port_t i2c_num)
{
    uint8_t temp_h=0,temp_l=0,humi_h,humi_l,crc_tmp,tmp;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | WRITE_BIT, ACK_VAL);
    vTaskDelay(10 / portTICK_RATE_MS);
    i2c_master_write_byte(cmd, 0x00, ACK_VAL);

    vTaskDelay(30 / portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | READ_BIT, ACK_VAL);
    vTaskDelay(10 / portTICK_RATE_MS);    
    i2c_master_read_byte(cmd, &humi_h, ACK_VAL);
    i2c_master_read_byte(cmd, &humi_l, ACK_VAL);
    i2c_master_read_byte(cmd, &temp_h, ACK_VAL);
    i2c_master_read_byte(cmd, &temp_l, ACK_VAL);
    i2c_master_read_byte(cmd, &crc_tmp, NACK_VAL);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    tmp = (uint8_t)(temp_h + temp_l + humi_h + humi_l);
		
	if(tmp == crc_tmp)
	{
		float temp = temp_h + temp_l / 10;
		float humi = humi_h + humi_l / 10;
			
        printf("温度: %0.2f 摄氏度, 湿度: %0.2f \n", temp, humi);
    }
	else
    {
        ESP_LOGI(TAG, "temp_h: %d", temp_h);
        ESP_LOGI(TAG, "temp_l: %d", temp_l);
        ESP_LOGI(TAG, "humi_h: %d", humi_h);
        ESP_LOGI(TAG, "humi_l: %d", humi_l);
        ESP_LOGI(TAG, "crc value: %d", crc_tmp);
            
        ESP_LOGI(TAG, "CRC Error");
	}

    return ret;
}

void app_main(void)
{
    gpioSetup(15, OUTPUT, LOW);//init led
    gpioSetup(12, OUTPUT, HIGH);//ir send

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    // ESP_LOGI(TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            // chip_info.cores,
            // (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            // (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    if (digitalLeds_initStrands(STRANDS, STRANDCNT)) {
		ESP_LOGI(TAG, "Init FAILURE: halting");
		while (true) {};
	}

    // i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM);
    // xTaskCreate(test_dht12_task, "test_dht12_task", 1024 * 2, (void* ) 1, 10, NULL);

    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_writer, mp3_decoder;
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // ESP_LOGI(TAG, "[ 1 ] Start audio codec chip");
#if CONFIG_ESP_M5CORE_NODE_BOARD
    audio_hal_codec_config_t audio_hal_codec_cfg = AUDIO_HAL_WM8978_DEFAULT();//for NODE Module supplied by M5Stack
    audio_hal_handle_t hal = audio_hal_init(&audio_hal_codec_cfg, 3);
#endif

    // audio_hal_ctrl_codec(hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    // ESP_LOGI(TAG, "[ 2 ] Create audio pipeline, add all elements to pipeline, and subscribe pipeline event");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    // ESP_LOGI(TAG, "[2.1] Create mp3 decoder to decode mp3 file and set custom read callback");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);
    audio_element_set_read_cb(mp3_decoder, mp3_music_read_cb, NULL);

    // ESP_LOGI(TAG, "[2.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.i2s_config.sample_rate = 48000;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    // ESP_LOGI(TAG, "[2.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    // ESP_LOGI(TAG, "[2.4] Link it together [mp3_music_read_cb]-->mp3_decoder-->i2s_stream-->[codec_chip]");
#if (CONFIG_ESP_LYRAT_V4_3_BOARD || CONFIG_ESP_LYRAT_V4_2_BOARD || CONFIG_ESP_M5CORE_NODE_BOARD) 
    audio_pipeline_link(pipeline, (const char *[]) {"mp3", "i2s"}, 2);
#endif

    /**Zl38063 does not support 44.1KHZ frequency, so resample needs to be used to convert files to other rates.
     * You can transfer to 16kHZ or 48kHZ.
     */
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 44100;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = 48000;
    rsp_cfg.dest_ch = 2;
    rsp_cfg.type = AUDIO_CODEC_TYPE_DECODER;
    audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);
    audio_pipeline_register(pipeline, filter, "filter");
    audio_pipeline_link(pipeline, (const char *[]) {"mp3", "filter", "i2s"}, 3);
#endif
    // ESP_LOGI(TAG, "[ 3 ] Setup event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    // ESP_LOGI(TAG, "[3.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    // ESP_LOGI(TAG, "[ 4 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            // ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) mp3_decoder
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(mp3_decoder, &music_info);

            // ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
            //          music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_setinfo(i2s_stream_writer, &music_info);

            /* Es8388 and es8374 use this function to set I2S and codec to the same frequency as the music file, and zl38063
             * does not need this step because the data has been resampled.*/
        #if (CONFIG_ESP_LYRAT_V4_3_BOARD || CONFIG_ESP_LYRAT_V4_2_BOARD || CONFIG_ESP_M5CORE_NODE_BOARD)
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates , music_info.bits, music_info.channels);
        #endif
            continue;
        }
        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && (int) msg.data == AEL_STATUS_STATE_STOPPED) {
            break;
        }
    }

    // ESP_LOGI(TAG, "[ 5 ] Stop audio_pipeline");
    audio_pipeline_terminate(pipeline);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener is called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, mp3_decoder);
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    audio_pipeline_unregister(pipeline, filter);
    audio_element_deinit(filter);
#endif
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(mp3_decoder);

    
    i2c_example_master_init();
        
    gpio_config_t io_conf;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    printf("\n");
    printf("\n");
    printf("开始测试 RGB 温湿度传感器 红外接收管....\n");
    printf("\n");
    printf("\n");

    while(1){
        // ESP_LOGI(TAG, "button c status: %d", gpio_get_level(GPIO_INPUT_IO_0));
        if(!gpio_get_level(GPIO_INPUT_IO_0))
        {
            //pressed
            printf("\n");
            printf("测试 RGB 和 温湿度传感器\n");
            set_white_color();
            i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM);
            
        }
        if(!gpio_get_level(GPIO_INPUT_IO_1))
        {
            //pressed
            set_black_color();
            if(!gpio_get_level(GPIO_INPUT_IO_2))
            {
                printf("接收到红外\n");
            }
            // else{
            //     ESP_LOGI(TAG, "不能接收到红外");
            // }
            // ESP_LOGI(TAG, "测试 红外接收");
        }
        
        vTaskDelay(200 / portTICK_RATE_MS); 
    }
}
