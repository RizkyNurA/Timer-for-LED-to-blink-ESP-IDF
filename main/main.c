#include <stdio.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define Pin_Button 22
#define Pin_LED 23
#define ADC_Unit ADC_UNIT_1
#define Pin_Potensio ADC_CHANNEL_0

static void periodic_timer_callback(void* arg);
esp_timer_handle_t periodic_timer;

static void oneshot_timer_callback(void* arg);
esp_timer_handle_t oneshot_timer;


static const char* TAG = "example";

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_chan0_handle;
bool do_calibration1_chan0;

static bool led_blinking = false;
static bool periodic_running = false;

void GPIO_Initialation(){

    gpio_reset_pin(Pin_Button);
    gpio_set_direction(Pin_Button, GPIO_MODE_INPUT);
    gpio_set_pull_mode(Pin_Button, GPIO_PULLUP_ONLY);

    gpio_reset_pin(Pin_LED);
    gpio_set_direction(Pin_LED, GPIO_MODE_OUTPUT);
}

void ADC_Initialation(int unit, int channel){
    //-------------ADC1 Init---------------//
    //adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = unit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = 3,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));

    //-------------ADC Calibration Init---------------//
    do_calibration1_chan0 = example_adc_calibration_init(unit, channel, 3, &adc1_cali_chan0_handle);
    
}

void create_periodic_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */
}

void create_oneshot_timer()
{
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &oneshot_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = (void*) periodic_timer,
            .name = "one-shot"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
}

void start_periodic_timer(int callback_time)
{
    /* Start the timers */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, callback_time));
    ESP_LOGI(TAG, "Started timers, time since boot: %lld us", esp_timer_get_time());
}

void start_oneshot_timer(int callback_time)
{
    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, callback_time));
}


void read_ADC(int unit, int channel)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw[0][0]));
    //ESP_LOGI("LOG", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw[0][0]);
    if (do_calibration1_chan0) 
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI("LOG", "ADC%d Channel[%d] Cali Voltage: %d mV", unit + 1, channel, voltage[0][0]);
    }
}

int button_pressed(int state){
	static int last_state = 0;
	int ret = 0;

	// Detect rising edge(button just pressed)
	if (!last_state&& state){
		ret = 1;
	}
	
	last_state = state;
	return ret;
}

void led_blinking_start(int period_us)
{
    led_blinking = true;
    if (periodic_running)
    {
        ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    }
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, period_us));
    periodic_running = true;
}

void led_blinking_stop()
{
    led_blinking = false;
    if (periodic_running)
    {
        ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
        periodic_running = false;
    }
    gpio_set_level(Pin_LED, 0);
}

void app_main(void)
{
    create_oneshot_timer();
    create_periodic_timer();
    GPIO_Initialation();
    ADC_Initialation(ADC_Unit, Pin_Potensio);
    
    
    static int last_adc_time = 0;

    while(1)
    {
        int button_state = gpio_get_level(Pin_Button);

        if (esp_timer_get_time() - last_adc_time > 1000000) 
        {
            read_ADC(ADC_Unit, Pin_Potensio);
            last_adc_time = esp_timer_get_time();
        }

        if (button_pressed(button_state))
        {

            esp_timer_stop(oneshot_timer);
            led_blinking_start(500000);
            start_oneshot_timer(5000000);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

static void periodic_timer_callback(void* arg)
{
    static bool level = 0;

    if (led_blinking)
    {
        level = !level;
        gpio_set_level(Pin_LED, level);
    }

}

static void oneshot_timer_callback(void* arg)
{
    led_blinking_stop();
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("Calibration", "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("Calibration", "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI("Calibration", "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW("Calibration", "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE("Calibration", "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI("Calibration", "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI("Calibration", "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}