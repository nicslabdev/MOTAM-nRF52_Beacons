/***************************************************************************************/
/*
 * intelligent_traffic_light
 * Created by Manuel Montenegro, Oct 10, 2018.
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
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_radio_notification.h"

#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"


// ======== Global configuration and functions ========

#define RED_STATE						0x00									// Identifier of Red traffic light state
#define YELLOW_STATE					0x01									// Identifier of Yellow traffic light state
#define GREEN_STATE						0x02									// Identifier of Green traffic light state
#define RED_STATE_DURATION				8										// Duration of red traffic light state in seconds
#define YELLOW_STATE_DURATION			4										// Duration of yellow traffic light state in seconds
#define GREEN_STATE_DURATION			8										// Duration of green traffic light state in seconds
static uint8_t 							lastState;								// Last traffic light state
static uint8_t							currentState;							// Current traffic light state
static uint32_t 						lastStateTicks;							// Timers ticks of the last state change

static uint32_t timer_ticks_to_ms (uint32_t ticks);
static uint8_t current_state_duration ();


// ======== BLE configuration ========

// MOTAM Beacon static parameters
#define STATIC_PARAMETERS_LENGTH		0x12									// Length of ADV_DATA_TYPE, MOTAM_ID, LATITUDE AND LONGITUDE
#define ADV_DATA_TYPE					0xFF									// Advertising data type (0xFF -> Manufacturer specific data)
#define MOTAM_ID                   		0xBE, 0xDE								// MOTAM identifier (DEphisit BEacon in little endian)
#define LATITUDE						0xA9, 0xDA, 0xE3, 0x41					// GPS latitude of the beacon (float in little endian)
#define LONGITUDE						0xBC, 0x98, 0x82, 0xC1					// GPS longitude of the beacon (float in little endian)
#define DIRECTION_FROM					0x23, 0X00								// From direction that applies (35)
#define DIRECTION_TO					0x18, 0x01								// To direction that applies (280)
#define BEACON_TYPE						0x04									// Type of MOTAM beacon (0x04 -> Intelligent Traffic Light)
#define DEFAULT_STATE					0xFF									// Current state (0x00 red, 0x01 yellow, 0x02 green, 0xFF undeterminated)
#define DEFAULT_TIMEOUT					0x00									// Time left for next state (0x00 -> no timeout)

// BLE configuration parameters
#define APP_BLE_CONN_CFG_TAG			1										// Tag that identifies the BLE configuration of the SoftDevice
#define APP_BLE_OBSERVER_PRIO			3										// BLE observer priority of the application. There is no need to modify this value
#define ADV_INTERVAL    				MSEC_TO_UNITS(100, UNIT_0_625_MS)  		// The advertising interval for non-connectable advertisement

static ble_gap_adv_params_t 			m_adv_params;							// Parameters to be passed to the stack when starting advertising
static ble_gap_adv_data_t 				m_adv_data;								// Struct that contains pointers to the BLE encoded advertising data
static uint8_t          				m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;	// Advertising handle used to identify an advertising set
static uint8_t 							m_enc_advdata_1 [BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED];	// Buffer for storing encoded advertising data
static uint8_t 							m_enc_advdata_2 [BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED];	// Buffer for storing encoded advertising data

static uint8_t frame [] =														// Intelligent traffic light default frame
	{
		STATIC_PARAMETERS_LENGTH,
		ADV_DATA_TYPE,
		MOTAM_ID,
		LATITUDE,
		LONGITUDE,
		DIRECTION_FROM,
		DIRECTION_TO,
		BEACON_TYPE,
		DEFAULT_STATE,
		DEFAULT_TIMEOUT
	};

// BLE events handler declaration
static void ble_evt_handler (ble_evt_t const * p_ble_evt, void * p_context);
static void frame_init (void);

// Function for initializing the BLE stack
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

// Function for initializing the Advertising functionality
static void advertising_init(void)
{
	ret_code_t err_code;

	// Set the advertising parameters
	m_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;	// Extended advertising
	m_adv_params.p_peer_addr     = NULL;    									// Undirected advertisement
	m_adv_params.interval        = ADV_INTERVAL;								// Time between advertisements
	m_adv_params.duration        = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;   	// Never time out
	m_adv_params.max_adv_evts	 = 0;											// No limit of advertising events
	m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;							// Allow scan request from any device
	m_adv_params.primary_phy 	 = BLE_GAP_PHY_CODED;							// Long range codification
	m_adv_params.secondary_phy 	 = BLE_GAP_PHY_CODED;							// Long range codification

	// Set the advertising data
	m_adv_data.adv_data.len = 0;												// No data on frame still
	m_adv_data.adv_data.p_data = m_enc_advdata_1;
	frame_init();

	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
	APP_ERROR_CHECK(err_code);
}

// Build beacon frame with MOTAM beacon structure
static void frame_init (void)
{
	memset(m_adv_data.adv_data.p_data, 0, BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED);
	m_adv_data.adv_data.len = sizeof(frame);
	memcpy (m_adv_data.adv_data.p_data, frame, sizeof(frame));

}

// Function for starting advertising
static void advertising_start(void)
{
	ret_code_t err_code;
	err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
	APP_ERROR_CHECK(err_code);
}

// Function for updating advertising data
static void advertising_update (uint8_t * new_frame, uint8_t new_frame_size)
{
	ret_code_t err_code;

	static uint8_t advdata_flag = 1;											// Last enc_advdata used. Necessary in order to dinamically update adv data

	if (advdata_flag == 1)
		{
		memcpy (m_enc_advdata_2, new_frame, new_frame_size);
		m_adv_data.adv_data.p_data = m_enc_advdata_2;
		advdata_flag = 2;
	}
	else
	{
		memcpy (m_enc_advdata_1, new_frame, new_frame_size);
		m_adv_data.adv_data.p_data = m_enc_advdata_1;
		advdata_flag = 1;
	}

	m_adv_data.adv_data.len = new_frame_size;

	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, NULL);
	APP_ERROR_CHECK(err_code);
}

// BLE events handler
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        	break;

        default:
            break;
    }
}

static void radio_active_handler ( bool radio_active );

// Radio notification will notify when an advertisement is sent
static void radio_notification_init (void)
{
    uint32_t err_code;

    err_code = ble_radio_notification_init
    		(
			APP_IRQ_PRIORITY_LOW,
			NRF_RADIO_NOTIFICATION_DISTANCE_800US,
			radio_active_handler
    		);
    APP_ERROR_CHECK(err_code);
}

// Handler for radio notification interruption (radio active and nonactive)
static void radio_active_handler ( bool radio_active )
{
	if (radio_active == false)
	{
		uint8_t timeout = current_state_duration() - (timer_ticks_to_ms (app_timer_cnt_diff_compute (app_timer_cnt_get(),lastStateTicks))/1000);
		frame[17] = currentState;
		frame[18] = timeout;
		advertising_update (frame, sizeof(frame));
	}
}


// ======== GPIO and timers initialization ========

// GPIO LED parameters
#define LED_RED NRF_GPIO_PIN_MAP(1,10)
#define LED_YELLOW NRF_GPIO_PIN_MAP(1,13)
#define LED_GREEN NRF_GPIO_PIN_MAP(1,15)

#define DURATION_RED APP_TIMER_TICKS(8000)										// Duration of red state in milliseconds
#define DURATION_YELLOW APP_TIMER_TICKS(4000)									// Duration of yellow state in milliseconds
#define DURATION_GREEN APP_TIMER_TICKS(8000)									// Duration of green state in mmilliseconds

APP_TIMER_DEF (timer);															// Timer instance

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
	nrf_gpio_cfg(																// Set up of Red Light
			LED_RED,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_H0H1,
			NRF_GPIO_PIN_NOSENSE
			);
	nrf_gpio_cfg(																// Set up of Yellow Light
			LED_YELLOW,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_H0H1,
			NRF_GPIO_PIN_NOSENSE
			);
	nrf_gpio_cfg(																// Set up of Green Light
			LED_GREEN,
			NRF_GPIO_PIN_DIR_OUTPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_H0H1,
			NRF_GPIO_PIN_NOSENSE
			);

	pins_clear();																// Set all leds to low
}

// Handle the timers: this will change the state of the traffic light
static void timer_handler (void *p_context)
{
	ret_code_t err_code;

	pins_clear();																// Set all leds to low

	if (lastState == RED_STATE)
	{
		currentState = GREEN_STATE;												// New state: green
		nrf_gpio_pin_set(LED_GREEN);											// Turn on green light
		err_code = app_timer_start ( timer, DURATION_GREEN, NULL);
		APP_ERROR_CHECK(err_code);
	}
	else if (lastState == GREEN_STATE)
	{
		currentState = YELLOW_STATE;											// New state: yellow
		nrf_gpio_pin_set(LED_YELLOW);											// Turn on green light
		err_code = app_timer_start ( timer, DURATION_YELLOW, NULL);
		APP_ERROR_CHECK(err_code);
	}
	else if (lastState == YELLOW_STATE)
	{
		currentState = RED_STATE;												// New state: red
		nrf_gpio_pin_set(LED_RED);												// Turn on red light
		err_code = app_timer_start ( timer, DURATION_RED, NULL);
		APP_ERROR_CHECK(err_code);
	}

	lastState = currentState;
	lastStateTicks = app_timer_cnt_get();
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

// Convert timer ticks to miliseconds
static uint32_t timer_ticks_to_ms (uint32_t ticks)
{
	return (ticks * 1000 * ((APP_TIMER_CONFIG_RTC_FREQUENCY) +1)) / APP_TIMER_CLOCK_FREQ;
}


// ======== Auxiliary functions ========

// Print beacon info by serial port
static void print_beacon_info (void)
{
    NRF_LOG_RAW_INFO("-------- INTELLIGENT TRAFFIC LIGHT BEACON --------\r\n");
    NRF_LOG_RAW_INFO("Latitude (little-end): 0x%02x 0x%02x 0x%02x 0x%02x\r\n", LATITUDE);
    NRF_LOG_RAW_INFO("Longitude (little-end): 0x%02x 0x%02x 0x%02x 0x%02x\r\n", LONGITUDE);
    NRF_LOG_RAW_INFO("Direction from (little-end): 0x%02x 0x%02x\r\n", DIRECTION_FROM);
    NRF_LOG_RAW_INFO("Direction to (little-end): 0x%02x 0x%02x\r\n", DIRECTION_TO);
    NRF_LOG_RAW_INFO("Beacon type: 0x%02x\r\n", BEACON_TYPE);
    NRF_LOG_RAW_INFO("--------------------------------------------------\r\n");
}

// Returns the duration of current traffic light state in seconds
static uint8_t current_state_duration ( )
{
	if (currentState == RED_STATE)
		return RED_STATE_DURATION;
	else if (currentState == YELLOW_STATE)
		return YELLOW_STATE_DURATION;
	else if (currentState == GREEN_STATE)
		return GREEN_STATE_DURATION;
	else
		return 0;
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
    ble_stack_init();
    advertising_init();
    radio_notification_init();

    // Start execution.
    lastState = YELLOW_STATE;													// Initial traffic light state
    timers_start();
    advertising_start();
    print_beacon_info();

    // Enter main loop.
    for (;;)
    {
    	idle_state_handle();
    }
}
