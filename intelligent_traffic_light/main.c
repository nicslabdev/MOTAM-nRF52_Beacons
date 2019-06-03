/***************************************************************************************/
/*
 * intelligent_traffic_light
 * Created by Manuel Montenegro, May 29, 2019.
 * Developed for MOTAM project.
 *
 *  This is a secure connected traffic light. The management connection with LTE-M modem 
 *  (Particle Boron LTE) is over serial pins. The traffic light state is broadcasted over
 *  long range BLE. BLE authentication has been developed using CC310 crypto functions.
 *
 *  This code has been developed for Nordic Semiconductor nRF52840 PDK & nRF52840 dongle.
*/
/***************************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>

#include "sdk_common.h"

#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_radio_notification.h"

#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"

#include "mem_manager.h"

#include "nrf_crypto.h"
#include "nrf_crypto_error.h"
#include "nrf_crypto_ecc.h"
#include "nrf_crypto_ecdsa.h"


// ======== Global configuration and functions ========

#define RED_STATE                               0x01                                        // Identifier of Red traffic light state
#define YELLOW_STATE                            0x02                                        // Identifier of Yellow traffic light state
#define GREEN_STATE                             0x03                                        // Identifier of Green traffic light state
#define EMERGENCY_RED_STATE                     0x04                                        // Identifier of red state due to a emergency vehicle is near
#define EMERGENCY_GREEN_STATE                   0x05                                        // Identifier of green state due to a emergency vehicle is near
#define RED_STATE_DURATION                      5                                           // Duration of red traffic light state in seconds (MINIMUM VALUE: 5 SECONDS)
#define YELLOW_STATE_DURATION                   1                                           // Duration of yellow traffic light state in seconds
#define GREEN_STATE_DURATION                    5                                           // Duration of green traffic light state in seconds
#define BLINKY_STATE_DURATION                   1                                           // Duration of blinky state of pedestrian green light
static uint8_t                                  lastState;                                  // Last traffic light state
static uint8_t                                  currentState;                               // Current traffic light state
static uint32_t                                 lastStateTicks;                             // Timers ticks of the last state change

static uint32_t timer_ticks_to_ms (uint32_t ticks);
static uint8_t current_state_duration ();


// ======== BLE configuration ========

// MOTAM Beacon default advertising frame
#define BEACON_LENGTH_FIELD                     86                                          // Length of advertising data (see BLE documentation)
#define ADV_DATA_TYPE                           0xFF                                        // Advertising data type (0xFF -> Manufacturer specific data)
#define MOTAM_ID                                0xBE, 0x5E                                  // MOTAM identifier (5Ecure BEacon)
#define DEFAULT_TIMESTAMP                       0x00, 0x00, 0x00, 0x00                      // Default UNIX timestamp
#define BEACON_TYPE                             0x04                                        // Type of MOTAM beacon (0x04 -> Intelligent Traffic Light)
#define LATITUDE                                0xA9, 0xDA, 0xE3, 0x41                      // GPS latitude of the beacon (float in little endian)
#define LONGITUDE                               0xBC, 0x98, 0x82, 0xC1                      // GPS longitude of the beacon (float in little endian)
#define DEVICE_ID                               LATITUDE, LONGITUDE                         // Device ID on traffic light corresponds the gps coordinates
#define DIRECTION_FROM                          0x23, 0X00                                  // From direction that applies (35)
#define DIRECTION_TO                            0x18, 0x01                                  // To direction that applies (280)
#define DEFAULT_STATE                           0xFF                                        // Current state (0x00 red, 0x01 yellow, 0x02 green, 0xFF undeterminated)
#define DEFAULT_TIMELEFT                        0X00                                        // Time left for next state (0x00 -> no timeout)
                                                                                            // Signature of default frame (SECP256K1)
#define DEFAULT_SIGN                            0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, \
                                                0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, \
                                                0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, \
                                                0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, \
                                                0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, \
                                                0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, \
                                                0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, \
                                                0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51

// BLE configuration parameters
#define APP_BLE_CONN_CFG_TAG                    1                                           // Tag that identifies the BLE configuration of the SoftDevice
#define APP_BLE_OBSERVER_PRIO                   3                                           // BLE observer priority of the application. There is no need to modify this value
#define ADV_INTERVAL                            MSEC_TO_UNITS(100, UNIT_0_625_MS)           // The advertising interval for non-connectable advertisement

static ble_gap_adv_params_t                     m_adv_params;                               // Parameters to be passed to the stack when starting advertising
static ble_gap_adv_data_t                       m_adv_data;                                 // Struct that contains pointers to the BLE encoded advertising data
static uint8_t                                  m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;  // Advertising handle used to identify an advertising set
static uint8_t                                  m_enc_advdata_1 [BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED]; // Buffer for storing encoded advertising data
static uint8_t                                  m_enc_advdata_2 [BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED]; // Buffer for storing encoded advertising data

// Frame that will be broadcasted by BLE advertising
#define BEACON_FRAME_LENGTH                     BEACON_LENGTH_FIELD + 1                     // Length of MOTAM advertising frame
static uint8_t frame [BEACON_FRAME_LENGTH] =                                                // Intelligent traffic light broadcast frame
        {
            BEACON_LENGTH_FIELD,
            ADV_DATA_TYPE,
            MOTAM_ID,
            DEFAULT_TIMESTAMP,
            BEACON_TYPE,
            DEVICE_ID,
            DIRECTION_FROM,
            DIRECTION_TO,
            DEFAULT_STATE,
            DEFAULT_TIMELEFT,
            DEFAULT_SIGN
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
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;  // Extended advertising
    m_adv_params.p_peer_addr    = NULL;                                                    // Undirected advertisement
    m_adv_params.interval       = ADV_INTERVAL;                                            // Time between advertisements
    m_adv_params.duration       = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;                   // Never time out
    m_adv_params.max_adv_evts   = 0;                                                       // No limit of advertising events
    m_adv_params.filter_policy  = BLE_GAP_ADV_FP_ANY;                                      // Allow scan request from any device
    m_adv_params.primary_phy    = BLE_GAP_PHY_CODED;                                       // Long range codification
    m_adv_params.secondary_phy  = BLE_GAP_PHY_CODED;                                       // Long range codification

    // Set the advertising data
    m_adv_data.adv_data.len = 0;                                                            // Still no data on frame
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

    static uint8_t advdata_flag = 1;                                                        // Last enc_advdata used. 
                                                                                            // Necessary in order to dinamically update adv data
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
        advertising_update (frame, sizeof(frame));
    }
}


// ======== Crypto configuration ========

// Intelligent Traffic Light private key in byte array format
static const uint8_t private_key_byte_array [] =
{
    0x80, 0xD2, 0xEF, 0x9D, 0x30, 0x4D, 0x6A, 0x84,
    0x04, 0x53, 0x00, 0xFA, 0x4A, 0x61, 0x46, 0xB3,
    0x7D, 0xB2, 0x19, 0xCA, 0x2F, 0x9D, 0x25, 0x52,
    0x75, 0xCA, 0x62, 0x58, 0x24, 0x43, 0xC5, 0x74,
};


// Intelligent Traffic Light public key in byte array format
static const uint8_t public_key_byte_array[] =
{
    0x40, 0xCD, 0xA5, 0xA4, 0xFF, 0x1E, 0xA3, 0xAE,
    0xCC, 0x67, 0x8E, 0xED, 0xF6, 0x36, 0x34, 0x9D,
    0x5B, 0x2D, 0xB7, 0x86, 0xAC, 0xA0, 0xD6, 0xA0,
    0xFB, 0x0F, 0xC9, 0xAE, 0x3A, 0x39, 0x14, 0x34,
    0xDE, 0x52, 0xEE, 0xAC, 0xA8, 0xE0, 0xCD, 0xC3,
    0x0C, 0xFA, 0xC6, 0xCF, 0x4D, 0xBF, 0x54, 0xDC,
    0x9F, 0x43, 0x9A, 0xE8, 0x9A, 0x48, 0xA0, 0x8D,
    0x87, 0xB1, 0xF1, 0x95, 0xE0, 0x50, 0x87, 0x45,
};



static nrf_crypto_ecc_private_key_t             private_key;                                // Intelligent Traffic Light private key in internal representation
static nrf_crypto_ecc_public_key_t              public_key;                                 // Intelligent Traffic Light public key in internal representation
static nrf_crypto_ecdsa_secp256k1_signature_t   signature;                                  // Advertising frame data ECDSA signature
static size_t                                   signature_size;

// Function for initializing the crypto module
static void crypto_init ()
{
    ret_code_t err_code;

    err_code = nrf_crypto_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_ecc_private_key_from_raw
        (
        &g_nrf_crypto_ecc_secp256k1_curve_info,
        &private_key,
        private_key_byte_array,
        sizeof(private_key_byte_array)
        );

    APP_ERROR_CHECK(err_code);
}


// Function for generating ECDSA advertising signature and include it on the frame
static void sign_frame ()
{
    ret_code_t err_code;

    nrf_crypto_hash_context_t hash_context;
    nrf_crypto_hash_sha256_digest_t hash_digest;
    size_t digest_size = sizeof(hash_digest);

    // Not all the frame must be signed. Discarding the signature field of advertising frame...
    uint8_t frame_data [BEACON_FRAME_LENGTH - NRF_CRYPTO_ECDSA_SECP256K1_SIGNATURE_SIZE];
    uint32_t frame_data_size = sizeof(frame_data);
    memcpy(frame_data,frame,frame_data_size);

    // It's necessary generating the hash of data before of sign
    err_code = nrf_crypto_hash_calculate
    (
        &hash_context,                                                                      // Context
        &g_nrf_crypto_hash_sha256_info,                                                     // Info structure
        frame_data,                                                                         // Input buffer
        frame_data_size,                                                                    // Input size
        hash_digest,                                                                        // Result buffer
        &digest_size                                                                        // Result size
    );
    APP_ERROR_CHECK(err_code);


    NRF_LOG_RAW_INFO("SIGN Frame data:\r\n");
    NRF_LOG_RAW_HEXDUMP_INFO(frame_data,frame_data_size);
    NRF_LOG_RAW_INFO("SIGN Generated hash:\r\n");
    NRF_LOG_RAW_HEXDUMP_INFO(hash_digest,digest_size);


    // Generate the signature of data
    signature_size = sizeof(signature);
    err_code = nrf_crypto_ecdsa_sign(NULL,
                                     &private_key,
                                     hash_digest,
                                     digest_size,
                                     signature,
                                     &signature_size);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_RAW_INFO("Generated signature:\r\n");
    NRF_LOG_RAW_HEXDUMP_INFO(signature,signature_size);
}


// ======== GPIO and timers initialization ========

// GPIO LED parameters
#define LED_RED                                 NRF_GPIO_PIN_MAP(0,13)
#define LED_YELLOW                              NRF_GPIO_PIN_MAP(0,14)
#define LED_GREEN                               NRF_GPIO_PIN_MAP(0,15)
//#define LED_RED                                 NRF_GPIO_PIN_MAP(1,10)
//#define LED_YELLOW                              NRF_GPIO_PIN_MAP(1,13)
//#define LED_GREEN                               NRF_GPIO_PIN_MAP(1,15)
#define LED_RED_PEDESTRIAN 			NRF_GPIO_PIN_MAP(0,29)
#define LED_GREEN_PEDESTRIAN			NRF_GPIO_PIN_MAP(0,31)


#define DURATION_RED 				APP_TIMER_TICKS(RED_STATE_DURATION*1000)    // Duration of red state in milliseconds
#define DURATION_YELLOW 			APP_TIMER_TICKS(YELLOW_STATE_DURATION*1000) // Duration of yellow state in milliseconds
#define DURATION_GREEN 				APP_TIMER_TICKS(GREEN_STATE_DURATION*1000)  // Duration of green state in milliseconds
#define DURATION_BLINKY				APP_TIMER_TICKS(BLINKY_STATE_DURATION*1000) // Duration of blinky in pedestrian green state in milliseconds

APP_TIMER_DEF (timer);                                                                      // Timer instance for traffic
APP_TIMER_DEF (timer_pedestrian);                                                           // Timer instance for pedestrian blinky green light

// Put all LEDs to low
static void pins_clear ()
{
    nrf_gpio_pin_clear(LED_RED);
    nrf_gpio_pin_clear(LED_YELLOW);
    nrf_gpio_pin_clear(LED_GREEN);
    nrf_gpio_pin_clear(LED_RED_PEDESTRIAN);
    nrf_gpio_pin_clear(LED_GREEN_PEDESTRIAN);
}


// GPIO ports initialization
static void gpio_init()
{
    nrf_gpio_cfg                                                                            // Set up of Red Light
    (
        LED_RED,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE
    );
    nrf_gpio_cfg                                                                            // Set up of Yellow Light
    (
        LED_YELLOW,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE
    );
    nrf_gpio_cfg                                                                            // Set up of Green Light
    (
        LED_GREEN,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE
    );
    nrf_gpio_cfg                                                                            // Set up of Green Light
    (
        LED_RED_PEDESTRIAN,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE
    );
    nrf_gpio_cfg                                                                            // Set up of Green Light
    (
        LED_GREEN_PEDESTRIAN,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE
    );

    pins_clear();                                                                           // Set all leds to low
}

// Handle the timers: this will change the state of the traffic light
static void timer_handler (void *p_context)
{
    ret_code_t err_code;

    pins_clear();                                                                           // Set all leds to low

    if (lastState == RED_STATE)
    {
        currentState = GREEN_STATE;                                                         // New state: green
        nrf_gpio_pin_set(LED_GREEN);                                                        // Turn on green light
        nrf_gpio_pin_set(LED_RED_PEDESTRIAN);
        err_code = app_timer_start ( timer, DURATION_GREEN, NULL);
        APP_ERROR_CHECK(err_code);
    }
    else if (lastState == GREEN_STATE)
    {
        currentState = YELLOW_STATE;                                                        // New state: yellow
        nrf_gpio_pin_set(LED_YELLOW);                                                       // Turn on green light
        nrf_gpio_pin_set(LED_RED_PEDESTRIAN);                                               // Turn on pedestrian red light
        err_code = app_timer_start ( timer, DURATION_YELLOW, NULL);
        APP_ERROR_CHECK(err_code);
    }
    else if (lastState == YELLOW_STATE)
    {
        currentState = RED_STATE;                                                           // New state: red
        nrf_gpio_pin_set(LED_RED);                                                          // Turn on red lightb
        nrf_gpio_pin_set(LED_GREEN_PEDESTRIAN);                                             // Turn on pedestrian green light
        err_code = app_timer_start (timer, DURATION_RED, NULL);
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_start (timer_pedestrian, (DURATION_RED-DURATION_BLINKY), NULL);// Start timer in order to start pedestrian blinky state
        APP_ERROR_CHECK(err_code);
    }

    lastState = currentState;
    lastStateTicks = app_timer_cnt_get();
}

// Handle the pedestrian timer: start blinky pedestrian green light
static void timer_handler_pedestrian (void *p_context)
{
    ret_code_t err_code;

    static int blink_cont = (int) BLINKY_STATE_DURATION / 0.25;                             // Number of cycles of turning on-turning off

    if (blink_cont > 0)
    {
        if (!(blink_cont%2))
        {
            nrf_gpio_pin_clear(LED_GREEN_PEDESTRIAN);
            blink_cont--;
            err_code = app_timer_start ( timer_pedestrian, APP_TIMER_TICKS(250), NULL);
            APP_ERROR_CHECK(err_code);
        }
        else if (blink_cont%2)
        {
            nrf_gpio_pin_set(LED_GREEN_PEDESTRIAN);
            blink_cont--;
            err_code = app_timer_start ( timer_pedestrian, APP_TIMER_TICKS(250), NULL);
            APP_ERROR_CHECK(err_code);
        }
    }
    else
    {
        blink_cont = (int) BLINKY_STATE_DURATION / 0.25;
    }
}


// Timer initialization
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create	( &timer, APP_TIMER_MODE_SINGLE_SHOT, timer_handler );
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create ( &timer_pedestrian, APP_TIMER_MODE_SINGLE_SHOT, timer_handler_pedestrian );
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
    NRF_LOG_RAW_INFO("\r\n-------- INTELLIGENT TRAFFIC LIGHT BEACON --------\r\n");
    NRF_LOG_RAW_INFO("Latitude (little-end): 0x%02x 0x%02x 0x%02x 0x%02x\r\n", LATITUDE);
    NRF_LOG_RAW_INFO("Longitude (little-end): 0x%02x 0x%02x 0x%02x 0x%02x\r\n", LONGITUDE);
    NRF_LOG_RAW_INFO("BLE advertising public key:\r\n");
    NRF_LOG_RAW_HEXDUMP_INFO(public_key_byte_array,sizeof(public_key_byte_array));
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

// Memory manager initialization
static void mem_init(void)
{
    ret_code_t err_code = nrf_mem_init();
    APP_ERROR_CHECK(err_code);
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
    // Start initialization
    log_init();
    clock_init();
    timers_init();
    power_management_init();
    gpio_init();
    ble_stack_init();
    advertising_init();
    radio_notification_init();

    mem_init();
    crypto_init();                                                                          // Crypto_init must be called AFTER ble_stack_init()!

    // Start execution.
    lastState = YELLOW_STATE;                                                               // Initial traffic light state
    timers_start();
    advertising_start();
    print_beacon_info();

    sign_frame();                                                                           // Generate ECDSA signature of frame and include it on it

    // Enter main loop.
    for (;;)
    {
    	idle_state_handle();
    }
}
