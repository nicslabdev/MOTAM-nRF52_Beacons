/***************************************************************************************/
/*
 * intelligent_traffic_light
 * Created by Manuel Montenegro, Oct 9, 2018.
 * Developed for MOTAM project.
 *
 *  This is a intelligent traffic light.
 *
 *  This code has been developed for Nordic Semiconductor nRF52840 PDK & nRF52840 dongle.
*/
/***************************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"


// ======== GPIO and timers initialization ========

#define LED_RED NRF_GPIO_PIN_MAP(1,10)
#define LED_YELLOW NRF_GPIO_PIN_MAP(1,13)
#define LED_GREEN NRF_GPIO_PIN_MAP(1,15)

#define DURATION_RED APP_TIMER_TICKS(4000)					// Duration of red state in milliseconds
#define DURATION_YELLOW APP_TIMER_TICKS(2000)				// Duration of yellow state in milliseconds
#define DURATION_GREEN APP_TIMER_TICKS(4000)				// Duration of green state in mmilliseconds

APP_TIMER_DEF (timer);										// Timer instance

typedef enum {red, yellow, green} tCurrentState;
tCurrentState currentState;									// Current traffic light state

// Put all LEDs to low
static void pins_clear ()
{
	nrf_gpio_pin_clear(LED_RED);
	nrf_gpio_pin_clear(LED_YELLOW);
	nrf_gpio_pin_clear(LED_GREEN);
}

// GPIO ports initialization
static void gpio_init()
{
	nrf_gpio_cfg(											// Set up of Red Light
			LED_RED,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_H0H1,
			NRF_GPIO_PIN_NOSENSE
			);
	nrf_gpio_cfg(											// Set up of Yellow Light
			LED_YELLOW,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_H0H1,
			NRF_GPIO_PIN_NOSENSE
			);
	nrf_gpio_cfg(											// Set up of Green Light
			LED_GREEN,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_H0H1,
			NRF_GPIO_PIN_NOSENSE
			);

	pins_clear();											// Set all leds to low
}

// Handle the timers: this will change the state of the traffic light
static void timer_handler (void *p_context)
{
	ret_code_t err_code;

	pins_clear();											// Set all leds to low

	if (currentState == red)
	{
		nrf_gpio_pin_set(LED_RED);
		err_code = app_timer_start ( timer, DURATION_RED, NULL);
		APP_ERROR_CHECK(err_code);
		currentState = green;
	}
	else if (currentState == green)
	{
		nrf_gpio_pin_set(LED_GREEN);
		err_code = app_timer_start ( timer, DURATION_GREEN, NULL);
		APP_ERROR_CHECK(err_code);
		currentState = yellow;
	}
	else if (currentState == yellow)
	{
		nrf_gpio_pin_set(LED_YELLOW);
		err_code = app_timer_start ( timer, DURATION_YELLOW, NULL);
		APP_ERROR_CHECK(err_code);
		currentState = red;
	}
}

// Timer initialization
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create	( &timer, APP_TIMER_MODE_SINGLE_SHOT, timer_handler );
    APP_ERROR_CHECK(err_code);
}

// Start the execution of concatenated timers
static void timers_start (void)
{
	timer_handler (NULL);
}


// ======== Board initialization ========

// Logging initialization
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

// Clock initialization
static void clock_init (void)
{
	ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }
}

// Power managemente initialization
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

// Function for handling the idle state (main loop)
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


int main(void)
{
    // Initialize
    log_init();
    clock_init();
    timers_init();
    power_management_init();
    gpio_init();

    // Start execution.
    NRF_LOG_RAW_INFO(    "  -------------------------------\r\n");
    NRF_LOG_RAW_INFO(	 "| MOTAM Intelligent Traffic Light |");
    NRF_LOG_RAW_INFO("\r\n  -------------------------------\r\n");

    currentState = red;										// Initial traffic light state
    timers_start();

    // Enter main loop.
    for (;;)
    {
    	idle_state_handle();
    }
}
