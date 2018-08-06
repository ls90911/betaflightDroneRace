# 1 "./src/main/drivers/resource.c"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "./src/main/drivers/resource.c"
# 21 "./src/main/drivers/resource.c"
# 1 "./src/main/drivers/resource.h" 1
# 21 "./src/main/drivers/resource.h"
       

typedef enum {
    OWNER_FREE = 0,
    OWNER_PWMINPUT,
    OWNER_PPMINPUT,
    OWNER_MOTOR,
    OWNER_SERVO,
    OWNER_LED,
    OWNER_ADC,
    OWNER_ADC_BATT,
    OWNER_ADC_CURR,
    OWNER_ADC_EXT,
    OWNER_ADC_RSSI,
    OWNER_SERIAL_TX,
    OWNER_SERIAL_RX,
    OWNER_PINDEBUG,
    OWNER_TIMER,
    OWNER_SONAR_TRIGGER,
    OWNER_SONAR_ECHO,
    OWNER_SYSTEM,
    OWNER_SPI_SCK,
    OWNER_SPI_MISO,
    OWNER_SPI_MOSI,
    OWNER_I2C_SCL,
    OWNER_I2C_SDA,
    OWNER_SDCARD,
    OWNER_SDCARD_CS,
    OWNER_SDCARD_DETECT,
    OWNER_FLASH_CS,
    OWNER_BARO_CS,
    OWNER_MPU_CS,
    OWNER_OSD_CS,
    OWNER_RX_SPI_CS,
    OWNER_SPI_CS,
    OWNER_MPU_EXTI,
    OWNER_BARO_EXTI,
    OWNER_COMPASS_EXTI,
    OWNER_USB,
    OWNER_USB_DETECT,
    OWNER_BEEPER,
    OWNER_OSD,
    OWNER_RX_BIND,
    OWNER_INVERTER,
    OWNER_LED_STRIP,
    OWNER_TRANSPONDER,
    OWNER_VTX,
    OWNER_COMPASS_CS,
    OWNER_RX_BIND_PLUG,
    OWNER_ESCSERIAL,
    OWNER_CAMERA_CONTROL,
    OWNER_TIMUP,
    OWNER_RANGEFINDER,
    OWNER_RX_SPI,
    OWNER_PINIO,
    OWNER_USB_MSC_PIN,
    OWNER_SPI_PREINIT_IPU,
    OWNER_SPI_PREINIT_OPU,
    OWNER_TOTAL_COUNT
} resourceOwner_e;

extern const char * const ownerNames[OWNER_TOTAL_COUNT];
# 22 "./src/main/drivers/resource.c" 2

const char * const ownerNames[OWNER_TOTAL_COUNT] = {
    "FREE",
    "PWM",
    "PPM",
    "MOTOR",
    "SERVO",
    "LED",
    "ADC",
    "ADC_BATT",
    "ADC_CURR",
    "ADC_EXT",
    "ADC_RSSI",
    "SERIAL_TX",
    "SERIAL_RX",
    "DEBUG",
    "TIMER",
    "SONAR_TRIGGER",
    "SONAR_ECHO",
    "SYSTEM",
    "SPI_SCK",
    "SPI_MISO",
    "SPI_MOSI",
    "I2C_SCL",
    "I2C_SDA",
    "SDCARD",
    "SDCARD_CS",
    "SDCARD_DETECT",
    "FLASH_CS",
    "BARO_CS",
    "MPU_CS",
    "OSD_CS",
    "RX_SPI_CS",
    "SPI_CS",
    "MPU_EXTI",
    "BARO_EXTI",
    "COMPASS_EXTI",
    "USB",
    "USB_DETECT",
    "BEEPER",
    "OSD",
    "RX_BIND",
    "INVERTER",
    "LED_STRIP",
    "TRANSPONDER",
    "VTX",
    "COMPASS_CS",
    "RX_BIND_PLUG",
    "ESCSERIAL",
    "CAMERA_CONTROL",
    "TIMUP",
    "RANGEFINDER",
    "RX_SPI",
    "PINIO",
    "USB_MSC_PIN",
    "SPI_PREINIT_IPU",
    "SPI_PREINIT_OPU",
};
