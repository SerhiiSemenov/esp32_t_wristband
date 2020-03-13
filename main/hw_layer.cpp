#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//#include "ahrs/MadgwickAHRS.h"
#include <esp_log.h>
#include "esp_system.h"
#include "driver/i2c.h"
#include "i2c_bus.hpp"
#include "mpu/math.hpp"
#include "tft/tftspi.h"
#include "tft/tft.h"
#include "spiffs/spiffs_vfs.h"
#include "hw_layer.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<LED_PIN) | (1ULL<<TP_PWR_PIN))
#define GPIO_INPUT_PIN_SEL  ((1ULL<<TP_PIN_PIN) | (1ULL<<IMU_INT_PIN) | (1ULL<<IMU_INT_PIN))

static const char* TAG = "HWLayer";
static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

HWLayer::HWLayer() : m_mpu{nullptr}
{

}

HWLayer::~HWLayer()
{

}

bool HWLayer::espSetup()
{
    //GPIO setup
    ESP_LOGI(TAG, "HW Initializing...");
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_config_t input_conf = {};
    //interrupt of rising edge
    input_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins
    input_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    input_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    input_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&input_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(TP_PIN_PIN, GPIO_INTR_ANYEDGE);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(TP_PIN_PIN, gpio_isr_handler, (void*) TP_PIN_PIN);

    gpio_set_level(TP_PWR_PIN, HIGH);
    gpio_set_level(LED_PIN, LOW);

    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, CLOCK_SPEED);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(gpio_num_t));

    return true;
}

void HWLayer::tftSetup()
{
    esp_err_t ret;
    tft_disp_type = DISP_TYPE_ST7735B;
    _width = DEFAULT_TFT_DISPLAY_WIDTH;  // smaller dimension
    _height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
    max_rdclock = 8000000;

    TFT_PinsInit();

    // ====  CONFIGURE SPI DEVICES(s)  ====================================================================================
    spi_lobo_device_handle_t spi;

    spi_lobo_bus_config_t buscfg={
       .mosi_io_num=PIN_NUM_MOSI,           // set SPI MOSI pin
       .miso_io_num=PIN_NUM_MISO,           // set SPI MISO pin
       .sclk_io_num=PIN_NUM_CLK,            // set SPI CLK pin
       .quadwp_io_num=-1,
       .quadhd_io_num=-1,
       .max_transfer_sz = 6*1024,
    };
    spi_lobo_device_interface_config_t devcfg={};
    devcfg.mode=0;                                // SPI mode 0
    devcfg.clock_speed_hz=8000000;                // Initial clock out at 8 MHz
    devcfg.spics_io_num=-1;                       // we will use external CS pin
    devcfg.spics_ext_io_num=PIN_NUM_CS;           // external CS pin
    devcfg.flags=LB_SPI_DEVICE_HALFDUPLEX;        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi

    vTaskDelay(500 / portTICK_RATE_MS);
    ESP_LOGI(TAG,"\r\n==============================");
    ESP_LOGI(TAG,"t-wristband demo\r\n");
    ESP_LOGI(TAG,"==============================");
    ESP_LOGI(TAG,"Pins used: miso=%d, mosi=%d, sck=%d, cs=%d\r\n", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
    ESP_LOGI(TAG,"TFT size %dx%d\r\n", _height, _width);
    ESP_LOGI(TAG,"==============================\r\n");

    ret=spi_lobo_bus_add_device(TFT_HSPI_HOST, &buscfg, &devcfg, &spi);
    assert(ret==ESP_OK);
    ESP_LOGI(TAG,"SPI: display device added to spi bus (%d)", TFT_HSPI_HOST);
    disp_spi = spi;

    // ==== Test select/deselect ====
    ret = spi_lobo_device_select(spi, 1);
    assert(ret==ESP_OK);
    ret = spi_lobo_device_deselect(spi);
    assert(ret==ESP_OK);

    ESP_LOGI(TAG,"SPI: attached display device, speed=%u", spi_lobo_get_speed(spi));
    ESP_LOGI(TAG,"SPI: bus uses native pins: %s", spi_lobo_uses_native_pins(spi) ? "true" : "false");

    ESP_LOGI(TAG,"SPI: display init...");
    TFT_display_init();
    ESP_LOGI(TAG,"OK\r\n");

    // ---- Detect maximum read speed ----
    max_rdclock = find_rd_speed();
    ESP_LOGI(TAG,"SPI: Max rd speed = %u", max_rdclock);

    // ==== Set SPI clock used for display operations ====
    spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
    ESP_LOGI(TAG,"SPI: Changed speed to %u", spi_lobo_get_speed(spi));

    ESP_LOGI(TAG,"\r\n---------------------");
    ESP_LOGI(TAG,"Graphics demo started");
    ESP_LOGI(TAG,"---------------------");

    font_rotate = 0;
    text_wrap = 0;
    font_transparent = 0;
    font_forceFixed = 0;
    gray_scale = 0;
    TFT_invertDisplay(INVERT_ON);
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
    TFT_setRotation(LANDSCAPE_FLIP);
    TFT_setFont(DEFAULT_FONT, NULL);
    TFT_resetclipwin();

    _fg = TFT_CYAN;
    TFT_print("Initializing SPIFFS...", CENTER, BOTTOM);
    // ==== Initialize the file system ====
    ESP_LOGI(TAG,"\r\n\n");
    vfs_spiffs_register();
    if (!get_spiffs_is_mounted()) {
       _fg = TFT_RED;
       TFT_print("SPIFFS not mounted !", CENTER, LASTY+TFT_getfontheight()+2);
    }
    else {
       _fg = TFT_GREEN;
       TFT_print("SPIFFS Mounted.", CENTER, LASTY+TFT_getfontheight()+2);
    }
    TFT_print("TEST STRING 1.", CENTER, CENTER);
    TFT_print("TEST STRING 2.", CENTER, LASTY+TFT_getfontheight()+2);
}

HWLayer::ButtonPress_t HWLayer::isButtonPresssed()
{
    gpio_num_t io_num;
    int val = 0;
    int start_time = 0;
    int end_time = 0;
    bool is_rising_catched = false;
    while (true) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            val = gpio_get_level(io_num);
            ESP_LOGI(TAG,"GPIO[%d] intr, val: %d, time %d\n", io_num, val, xTaskGetTickCount());
            if (val == 0) {
                end_time = xTaskGetTickCount();
                if (is_rising_catched & ((end_time - start_time) > 95)) {
                    return LONG_PRESS;
                }
                return SHORT_PRESS;
            }
            else {
                start_time = xTaskGetTickCount();
                is_rising_catched = true;
            }
        }
    }
    return NOT_PRESSED;//make compiler happy
}

void HWLayer::ledOn()
{
    gpio_set_level(LED_PIN, 1);
}

void HWLayer::ledOff()
{
    gpio_set_level(LED_PIN, 0);
}

void HWLayer::rtcSetup()
{
    esp_err_t ret = PCF_hctosys();
    if (ret != 0) {
        PCF_systohc();
        ESP_LOGI(TAG, "Error reading hardware clock: %d", ret);
    }
    ESP_LOGI(TAG,"ret %d time %ld\n ", (int)ret, time(NULL));
}

tm HWLayer::getTime()
{
    struct tm c_time = {};
    PCF_DateTime dateTime = {};
    PCF_GetDateTime(&dateTime);
    c_time.tm_sec = dateTime.second;
    c_time.tm_min = dateTime.minute;
    c_time.tm_hour = dateTime.hour;

    ESP_LOGI(TAG,"Time %02d:%02d:%02d\n", dateTime.hour, dateTime.minute, dateTime.second);
    return c_time;
}

void HWLayer::gyroSetup()
{
    m_mpu = new mpud::MPU();  // create a default MPU object
    m_mpu->setBus(i2c0);  // set bus port, not really needed since default is i2c0
    m_mpu->setAddr(mpud::MPU_I2CADDRESS_AD0_HIGH);  // set address, default is AD0_LOW

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    ESP_ERROR_CHECK(m_mpu->testConnection());
    ESP_LOGI(TAG, "MPU connection successful!");
    // Initialize
    ESP_ERROR_CHECK(m_mpu->initialize());  // initialize the chip and set initial configurations
}

void HWLayer::gyroTest()
{
    if (m_mpu != nullptr)
    {
        // Reading sensor data
        ESP_LOGI(TAG,"Reading sensor data:\n");
        mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
        mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
        mpud::float_axes_t accelG;   // accel axes in (g) gravity format
        mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
        // Read
        m_mpu->acceleration(&accelRaw);  // fetch raw data from the registers
        m_mpu->rotation(&gyroRaw);       // fetch raw data from the registers
        // MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
        // Convert
        accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        // Debug
        ESP_LOGI(TAG,"accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
        ESP_LOGI(TAG,"gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
    }
}

mpud::float_axes_t HWLayer::getGyro()
{
    mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
    m_mpu->rotation(&gyroRaw);       // fetch raw data from the registers
    return mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
}

mpud::float_axes_t HWLayer::getAccel()
{
    mpud::raw_axes_t accelRaw;       // x, y, z axes as int16
    m_mpu->acceleration(&accelRaw);  // fetch raw data from the registers
    return mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
}
