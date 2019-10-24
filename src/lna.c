/* main.c - Application main entry point */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>


#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(lna);

#define BT840F_LNA_GPIO_DEV "GPIO_0"
#define BT840F_APP_CHL_PIN  8 
#define BT840F_APP_CPS_PIN  6 
#define BT840F_APP_CHL_VAL  1 
#define BT840F_APP_CPS_VAL  0 

void lna_init(void)
{
    if (0 == strcmp(CONFIG_BOARD, "bt840f_dbg")){
        struct device * gpio_dev = device_get_binding(BT840F_LNA_GPIO_DEV);
        if (!gpio_dev){
            LOG_ERR("LNA/PA GPIO device \"%s\" not found",BT840F_LNA_GPIO_DEV);
        } else {
            gpio_pin_configure(gpio_dev, BT840F_APP_CHL_PIN, GPIO_DIR_OUT);
            gpio_pin_configure(gpio_dev, BT840F_APP_CPS_PIN, GPIO_DIR_OUT);
            gpio_pin_write(gpio_dev, BT840F_APP_CHL_PIN, BT840F_APP_CHL_VAL);
            gpio_pin_write(gpio_dev, BT840F_APP_CPS_PIN, BT840F_APP_CPS_VAL);
            LOG_INF("LNA/PA CHL pin (%d) is set to %d",BT840F_APP_CHL_PIN, BT840F_APP_CHL_VAL);
            LOG_INF("LNA/PA CPS pin (%d) is set to %d",BT840F_APP_CPS_PIN, BT840F_APP_CPS_VAL);
        }
    }

#ifdef CONFIG_BT_CTLR_GPIO_LNA        
        LOG_INF("LNA config: pin=%d, offset=%d, %s",
            CONFIG_BT_CTLR_GPIO_LNA_PIN, CONFIG_BT_CTLR_GPIO_LNA_OFFSET,
#if CONFIG_BT_CTLR_GPIO_LNA_POL_INV
        "invert"
#else
        "no invert"
#endif
            );
#else
        LOG_INF("LNA is not configured");
#endif
#ifdef CONFIG_BT_CTLR_GPIO_PA        
        LOG_INF("PA config: pin=%d, offset=%d, %s",
            CONFIG_BT_CTLR_GPIO_PA_PIN, CONFIG_BT_CTLR_GPIO_PA_OFFSET,
#if CONFIG_BT_CTLR_GPIO_PA_POL_INV
        "invert"
#else
        "no invert"
#endif
            );
#else
        LOG_INF("PA is not configured");
#endif
}
