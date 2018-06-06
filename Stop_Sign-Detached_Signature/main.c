/***************************************************************************************/
/*
 * Stop_Sign-Detached_Signature
 * Created by Manuel Montenegro, Jun 6, 2018.
 * Developed for MOTAM project.
 *
 *  This is an alternative to Stop_Sign application developed for devices that don't
 *  support extended advertisements. This beacon generates a secure signature and send
 *  two linked frames that include data and a secure digital signature (ECDSA
 *  cryptographic algorithm is used).
 *
 *  This code has been developed for Nordic Semiconductor nRF52840 PDK.
*/
/***************************************************************************************/


#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// [MOTAM] MOTAM Beacon static parameters
#define DATA_LENGTH                		17                                 	// [MOTAM] Size of this advertisement
#define ADV_DATA_TYPE					0xFF								// [MOTAM] Advertising data type (0xFF -> Manufacturer specific data)
#define MOTAM_ID                   		0xBE, 0xDE                        	// [MOTAM] MOTAM beacons' ID
#define LATITUDE						0xA9, 0xDA, 0xE3, 0x41				// [MOTAM] GPS latitude of the beacon (float in little endian)
#define LONGITUDE						0xBC, 0x98, 0x82, 0xC1				// [MOTAM] GPS longitude of the beacon (float in little endian)
#define BEACON_TYPE						0x01								// [MOTAM] Type of MOTAM beacon (0x01 -> Traffic sign)
#define SIGN_TYPE						0x10								// [MOTAM] Type of traffic signal (0x10 -> STOP)
#define DIRECTION_FROM					0x23, 0X00							// [MOTAM] From direction that applies (35)
#define DIRECTION_TO					0x18, 0x01							// [MOTAM] To direction that applies (280)

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */

static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */

static uint8_t				advdata_flag;									// [MOTAM] Flag: Last m_enc_advdata used. Necessary in order to update adv
static uint8_t              m_enc_advdata1[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static uint8_t              m_enc_advdata2[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; 	// [MOTAM] In order to update adv data, It has to alternate between the two buffers


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata1,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


// [MOTAM] Build beacon frame with MOTAM beacon structure
static void motam_frame_init ( )
{

	memset(m_adv_data.adv_data.p_data, 0, sizeof(m_adv_data.adv_data.p_data));

	uint8_t frame [] =
	{
			DATA_LENGTH,
			ADV_DATA_TYPE,
			MOTAM_ID,
			LATITUDE,
			LONGITUDE,
			BEACON_TYPE,
			SIGN_TYPE,
			DIRECTION_FROM,
			DIRECTION_TO
	};

	// [MOTAM] The frame contains data + 1 extra byte (DATA_LENGTH byte)
	m_adv_data.adv_data.len = DATA_LENGTH + 1;
	memcpy (m_adv_data.adv_data.p_data, frame, DATA_LENGTH + 1);

	advdata_flag = 1;

}


// [MOTAM] Advertising init function: configures advertising parameters for BLE 5
static void motam_advertising_init(void)
{
	ret_code_t err_code;

	// Initialize advertising parameters (used when starting advertising).
	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;	// [MOTAM] This is for Long Range codification (BLE_GAP_PHY_CODED)
	m_adv_params.p_peer_addr     = NULL;    			// Undirected advertisement.
	m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
	m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
	m_adv_params.duration        = 0;       			// Never time out.
	m_adv_params.primary_phy 	 = BLE_GAP_PHY_CODED;	// [MOTAM] Long range codification
	m_adv_params.secondary_phy 	 = BLE_GAP_PHY_CODED;	// [MOTAM] Long range codification

	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/*
 * [MOTAM] Update MOTAM beacon frame.
 * The frame size must be DATA_LENGTH + 1
*/
static void motam_frame_update ( uint8_t * frame, uint8_t frame_size )
{

	ret_code_t err_code;

	if (advdata_flag == 1)
	{
		memcpy (m_enc_advdata2, frame, frame_size);
		m_adv_data.adv_data.p_data = m_enc_advdata2;
		advdata_flag = 2;
	}
	else
	{
		memcpy (m_enc_advdata1, frame, frame_size);
		m_adv_data.adv_data.p_data = m_enc_advdata1;
		advdata_flag = 1;
	}

	m_adv_data.adv_data.len = frame_size;

	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, NULL);
	APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
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
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    log_init();    timers_init();    leds_init();    power_management_init();    ble_stack_init();

    motam_frame_init( );
    motam_advertising_init();
    advertising_start();

    NRF_LOG_INFO("-------------------------------------------------");
    NRF_LOG_INFO("MOTAM SIGN BEACON STARTED");
    NRF_LOG_INFO("Latitude (little-end): 0x%02x 0x%02x 0x%02x 0x%02x", LATITUDE);
    NRF_LOG_INFO("Longitude (little-end): 0x%02x 0x%02x 0x%02x 0x%02x", LONGITUDE);
    NRF_LOG_INFO("Beacon type: 0x%02x", BEACON_TYPE);
    NRF_LOG_INFO("Direction from applies (little-end): 0x%02x 0x%02x", DIRECTION_FROM);
    NRF_LOG_INFO("Direction to applies (little-end): 0x%02x 0x%02x", DIRECTION_TO);


    for (;;)
    {
        idle_state_handle();
    }
}

