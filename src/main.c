//#define KEYB 1
/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_hids_keyboard_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_keyboard
 * @brief HID Keyboard Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Services for implementing a simple keyboard functionality.
 * Pressing Button 0 will send text 'hello' to the connected peer. On receiving output report,
 * it toggles the state of LED 2 on the mother board based on whether or not Caps Lock is on.
 * This application uses the @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device.
 */

//#define USE_UART

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "nrf_adc.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#ifdef USE_UART
#include "app_uart.h"
#endif
#include "softdevice_handler_appsh.h"
#include "app_timer_appsh.h"
#include "device_manager.h"
#include "app_button.h"
#include "pstorage.h"
#include "app_trace.h"


#ifdef KEYB
static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern);
#else
static void send_gamepad();
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                              /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define UART_TX_BUF_SIZE 256                                                            /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                                                              /**< UART RX buffer size. */

#define KEY_PRESS_BUTTON_ID              0                                              /**< Button used as Keyboard key press. */
#define SHIFT_BUTTON_ID                  1                                              /**< Button used as 'SHIFT' Key. */

#ifdef KEYB
#define DEVICE_NAME                      "Nordic_Keyboard"                              /**< Name of device. Will be included in the advertising data. */
#else
#define DEVICE_NAME                      "Moga_joystick"                              /**< Name of device. Will be included in the advertising data. */
#endif

#define MANUFACTURER_NAME                "NordicSemiconductor"                          /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_TIMER_PRESCALER              0                                              /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                              /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)     /**< Battery level measurement interval (ticks). */
#define GAMEPAD_POLLING_INTERVAL         APP_TIMER_TICKS(1000/100, APP_TIMER_PRESCALER)     /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                81                                             /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                                            /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                              /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE          0x02                                           /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                 0x1915                                         /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                0xEEEE                                         /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION           0x0001                                         /**< Product Version. */

#define APP_ADV_FAST_INTERVAL            0x0028                                         /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL            0x0C80                                         /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT             30                                             /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT             180                                            /**< The duration of the slow advertising period (in seconds). */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(7.5, UNIT_1_25_MS)               /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(30, UNIT_1_25_MS)                /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                    6                                              /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(430, UNIT_10_MS)                 /**< Connection supervisory timeout (430 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)    /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                              /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                              /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                           /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                             /**< Maximum encryption key size. */

#define OUTPUT_REPORT_INDEX              0                                              /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN            1                                              /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX          0                                              /**< Index of Input Report. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK 0x02                                           /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */
#define INPUT_REP_REF_ID                 0                                              /**< Id of reference to Keyboard Input Report. */
#define OUTPUT_REP_REF_ID                0                                              /**< Id of reference to Keyboard Output Report. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2            /**< Reply when unsupported features are requested. */

#define MAX_BUFFER_ENTRIES               5                                              /**< Number of elements that can be enqueued */

#define BASE_USB_HID_SPEC_VERSION        0x0101                                         /**< Version number of base USB HID Specification implemented by this application. */
#ifdef KEYB
#define INPUT_REPORT_KEYS_MAX_LEN        8                                              /**< Maximum length of the Input Report characteristic. */
#else
#define INPUT_REPORT_KEYS_MAX_LEN        6                                              /**< Maximum length of the Input Report characteristic. */
#endif
#define DEAD_BEEF                        0xDEADBEEF                                     /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE        MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                             BLE_STACK_HANDLER_SCHED_EVT_SIZE)          /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                 10                                             /**< Maximum number of events in the scheduler queue. */

#define MODIFIER_KEY_POS                 0                                              /**< Position of the modifier byte in the Input Report. */
#define SCAN_CODE_POS                    2                                              /**< This macro indicates the start position of the key scan code in a HID Report. As per the document titled 'Device Class Definition for Human Interface Devices (HID) V1.11, each report shall have one modifier byte followed by a reserved constant byte and then the key scan code. */
#define SHIFT_KEY_CODE                   0x02                                           /**< Key code indicating the press of the Shift Key. */

#define MAX_KEYS_IN_ONE_REPORT           (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS)    /**< Maximum number of key presses that can be sent in one Input Report. */

#ifdef KEYB

/**Buffer queue access macros
 *
 * @{ */
/** Initialization of buffer list */
#define BUFFER_LIST_INIT()                                                                        \
        do                                                                                        \
        {                                                                                         \
            buffer_list.rp = 0;                                                                   \
            buffer_list.wp = 0;                                                                   \
            buffer_list.count = 0;                                                                \
        } while (0)

/** Provide status of data list is full or not */
#define BUFFER_LIST_FULL()\
        ((MAX_BUFFER_ENTRIES == buffer_list.count - 1) ? true : false)

/** Provides status of buffer list is empty or not */
#define BUFFER_LIST_EMPTY()\
        ((0 == buffer_list.count) ? true : false)

#define BUFFER_ELEMENT_INIT(i)\
        do                                                                                        \
        {                                                                                         \
            buffer_list.buffer[(i)].p_data = NULL;                                                \
        } while (0)
#endif
/** @} */

typedef enum
{
    BLE_NO_ADV,               /**< No advertising running. */
    BLE_DIRECTED_ADV,         /**< Direct advertising to the latest central. */
    BLE_FAST_ADV_WHITELIST,   /**< Advertising with whitelist. */
    BLE_FAST_ADV,             /**< Fast advertising running. */
    BLE_SLOW_ADV,             /**< Slow advertising running. */
    BLE_SLEEP,                /**< Go to system-off. */
} ble_advertising_mode_t;
#ifdef KEYB
/** Abstracts buffer element */
typedef struct hid_key_buffer
{
    uint8_t    data_offset;   /**< Max Data that can be buffered for all entries */
    uint8_t    data_len;      /**< Total length of data */
    uint8_t    * p_data;      /**< Scanned key pattern */
    ble_hids_t * p_instance;  /**< Identifies peer and service instance */
}buffer_entry_t;

STATIC_ASSERT(sizeof(buffer_entry_t) % 4 == 0);

/** Circular buffer list */
typedef struct
{
    buffer_entry_t buffer[MAX_BUFFER_ENTRIES]; /**< Maximum number of entries that can enqueued in the list */
    uint8_t        rp;                         /**< Index to the read location */
    uint8_t        wp;                         /**< Index to write location */
    uint8_t        count;                      /**< Number of elements in the list */
}buffer_list_t;

STATIC_ASSERT(sizeof(buffer_list_t) % 4 == 0);
#endif
static ble_hids_t                        m_hids;                                        /**< Structure used to identify the HID service. */
static ble_bas_t                         m_bas;                                         /**< Structure used to identify the battery service. */
static bool                              m_in_boot_mode = false;                        /**< Current protocol mode. */
static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;       /**< Handle of the current connection. */

#ifdef KEYB
static sensorsim_cfg_t                   m_battery_sim_cfg;                             /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t                 m_battery_sim_state;                           /**< Battery Level sensor simulator state. */
#endif

APP_TIMER_DEF(m_battery_timer_id);                                                      /**< Battery timer. */
APP_TIMER_DEF(m_gamepad_polling_timer_id);                                                      /**< Battery timer. */

static dm_application_instance_t         m_app_handle;                                  /**< Application identifier allocated by device manager. */
static dm_handle_t                       m_bonded_peer_handle;                          /**< Device reference handle to the current bonded central. */
#ifdef KEYB
static bool                              m_caps_on = false;                             /**< Variable to indicate if Caps Lock is turned on. */
#endif
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}};

#ifdef KEYB

static uint8_t m_sample_key_press_scan_str[] =                                          /**< Key pattern to be sent when the key press button has been pushed. */
{
    0x0b, /* Key h */
    0x08, /* Key e */
    0x0f, /* Key l */
    0x0f, /* Key l */
    0x12, /* Key o */
    0x28  /* Key Return */
};

static uint8_t m_caps_on_key_scan_str[] =                                                /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit set. */
{
    0x06, /* Key C */
    0x04, /* Key a */
    0x13, /* Key p */
    0x16, /* Key s */
    0x12, /* Key o */
    0x11, /* Key n */
};

static uint8_t m_caps_off_key_scan_str[] =                                               /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit cleared. */
{
    0x06, /* Key C */
    0x04, /* Key a */
    0x13, /* Key p */
    0x16, /* Key s */
    0x12, /* Key o */
    0x09, /* Key f */
};


/** List to enqueue not just data to be sent, but also related information like the handle, connection handle etc */
static buffer_list_t buffer_list;

#endif

#ifdef USE_UART

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            break;

        case APP_UART_COMMUNICATION_ERROR:
            break;

        case APP_UART_FIFO_ERROR:
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        18,//RX_PIN_NUMBER,
        20,// TX_PIN_NUMBER,
        0,
        0,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       1,
                       1,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    if (err_code!=0)
    //MY_APP_ERROR_CHECK(err_code);
    printf("hello uart\n");
}
#endif

#ifdef USE_UART
    #define MY_APP_ERROR_HANDLER(ERR_CODE)                      \
        do                                                      \
        {                                                       \
            printf("%08x line: %d %s\n", (unsigned int)ERR_CODE, __LINE__, __FILE__);                                 \
        } while (0)

    #define MY_APP_ERROR_CHECK(ERR_CODE)                       \
            if (ERR_CODE!=0) { printf("%08x line: %d %s\n", (unsigned int)ERR_CODE, __LINE__, __FILE__); }                                
#else
    #define MY_APP_ERROR_HANDLER(ERR_CODE)  \
       if (ERR_CODE!=0) do {} while(0);

    #define MY_APP_ERROR_CHECK(ERR_CODE) if (ERR_CODE!=0) {}
#endif


//--------------------------------------------
const uint32_t button_mask = 0xff << 5;

void configure_buttons()
{
    //nrf_gpiote_int_enable(button_mask);
    for (uint8_t i=0;i<32;i++)
    {
        if (((button_mask>>i)&1)==1) 
            nrf_gpio_cfg_input(i, GPIO_PIN_CNF_PULL_Pullup);
    }

}

uint8_t buttons_read()
{
    return (~((nrf_gpio_pins_read() & button_mask) >> 5)) & 0xff;
}

//void GPIOTE_IRQHandler()
//{
//    button_state = buttons_read();
//}

//--------------------------------------------

void adc_config()
{
    const nrf_adc_config_t nrf_adc_config = { NRF_ADC_CONFIG_RES_8BIT,               
                                 NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD, 
                                 NRF_ADC_CONFIG_REF_SUPPLY_ONE_THIRD };

    // Initialize and configure ADC
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
    //nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_1);
    //nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    //NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
    //NVIC_EnableIRQ(ADC_IRQn);
}

//--------------------------------------------

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);

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
#ifdef USE_UART
    printf("assert");
#endif

    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
#ifdef USE_UART
            printf("Disabled\n");
#endif            

    MY_APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    MY_APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level=100;

    //battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        MY_APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


bool check_gamepad = true;
static void gamepad_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    //if (check_gamepad == true)
    {
        send_gamepad();
    }
}

void gamepad_timer_start()
{
#ifdef USE_UART
    printf("%s\n", __FUNCTION__);
#endif
    uint32_t err_code = app_timer_start(m_gamepad_polling_timer_id, GAMEPAD_POLLING_INTERVAL, NULL);
    MY_APP_ERROR_CHECK(err_code);    
}


void gamepad_timer_stop()
{
#ifdef USE_UART
    printf("%s\n", __FUNCTION__);
#endif
    uint32_t err_code = app_timer_stop(m_gamepad_polling_timer_id);
    MY_APP_ERROR_CHECK(err_code);  
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    MY_APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_gamepad_polling_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                gamepad_timeout_handler);

    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    MY_APP_ERROR_CHECK(err_code);
#ifdef KEYB
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
#else
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_GAMEPAD);
#endif    
    MY_APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    uint32_t         err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    MY_APP_ERROR_CHECK(err_code);
}


static void on_battery_evt(ble_bas_t * p_bas, ble_bas_evt_t *p_evt)
{
#ifdef USE_UART
    printf("on_battery_evt: ");
#endif
    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
        {
#ifdef USE_UART
            printf("Disabled\n");
#endif
            uint32_t err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            MY_APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
        {
#ifdef USE_UART
            printf("Enabled\n");
#endif
            uint32_t err_code = app_timer_stop(m_battery_timer_id);
            MY_APP_ERROR_CHECK(err_code);
            break;
        }

        default:
#ifdef USE_UART
            printf("unknown\n");
#endif
            break;
    }
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = on_battery_evt;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    uint32_t                   err_code;
    ble_hids_init_t            hids_init_obj;
    ble_hids_inp_rep_init_t    input_report_array[1];
    ble_hids_inp_rep_init_t  * p_input_report;
#ifdef KEYB    
    ble_hids_outp_rep_init_t   output_report_array[1];
    ble_hids_outp_rep_init_t * p_output_report;
#endif    
    uint8_t                    hid_info_flags;

    memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));
#ifdef KEYB    
    memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));

    static uint8_t report_map_data[] =
    {
        0x05, 0x01,                 // Usage Page (Generic Desktop)
        0x09, 0x06,                 // Usage (Keyboard)
        0xA1, 0x01,                 // Collection (Application)
        0x05, 0x07,                 //     Usage Page (Key Codes)
        0x19, 0xe0,                 //     Usage Minimum (224)
        0x29, 0xe7,                 //     Usage Maximum (231)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x01,                 //     Logical Maximum (1)
        0x75, 0x01,                 //     Report Size (1)
        0x95, 0x08,                 //     Report Count (8)
        0x81, 0x02,                 //     Input (Data, Variable, Absolute)

        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x08,                 //     Report Size (8)
        0x81, 0x01,                 //     Input (Constant) reserved byte(1)

        0x95, 0x05,                 //     Report Count (5)
        0x75, 0x01,                 //     Report Size (1)
        0x05, 0x08,                 //     Usage Page (Page# for LEDs)
        0x19, 0x01,                 //     Usage Minimum (1)
        0x29, 0x05,                 //     Usage Maximum (5)
        0x91, 0x02,                 //     Output (Data, Variable, Absolute), Led report
        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x03,                 //     Report Size (3)
        0x91, 0x01,                 //     Output (Data, Variable, Absolute), Led report padding

        0x95, 0x06,                 //     Report Count (6)
        0x75, 0x08,                 //     Report Size (8)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x65,                 //     Logical Maximum (101)
        0x05, 0x07,                 //     Usage Page (Key codes)
        0x19, 0x00,                 //     Usage Minimum (0)
        0x29, 0x65,                 //     Usage Maximum (101)
        0x81, 0x00,                 //     Input (Data, Array) Key array(6 bytes)

        0x09, 0x05,                 //     Usage (Vendor Defined)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x26, 0xFF, 0x00,           //     Logical Maximum (255)
        0x75, 0x08,                 //     Report Count (2)
        0x95, 0x02,                 //     Report Size (8 bit)
        0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)

        0xC0                        // End Collection (Application)
    };
#else
    static uint8_t report_map_data[] = {
        0x05,   0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09,   0x05,                    // USAGE (Game Pad) - Hut1_12v2.pdf p28 of 128
        0xA1,   0x01,                    // COLLECTION (Application)

        0xA1,   0x00,                    //   COLLECTION (Physical)
//-----------------
        0x05,   0x01,                    //     USAGE_PAGE (Generic Desktop)
        0x09,   0x30,                    //     USAGE (X)
        0x09,   0x31,                    //     USAGE (Y)
        0x09,   0x32,                    //     USAGE (Z) - Hut1_12v2.pdf p26 = represents R X-axis
        0x09,   0x33,                    //     USAGE (Rx) - Hut1_12v2.pdf p26 = represents R Y-axis

        0x15,   0x81,                    //     LOGICAL_MINIMUM (-127)
        0x25,   0x7F,                    //     LOGICAL_MAXIMUM (127)
        0x75,   0x08,                    //     REPORT_SIZE (8)
        0x95,   0x04,                    //     REPORT_COUNT (4)

        0x81,   0x02,                    //     INPUT (Data,Var,Abs) - absolute for joysticks ( != rel for mouse )
//-----------------
        0x05,   0x09,                    //     USAGE_PAGE (Button)
        0x19,   0x01,                    //     USAGE_MINIMUM (Button 1)
        0x29,   0x10,                    //     USAGE_MAXIMUM (Button 16)

        0x15,   0x00,                    //     LOGICAL_MINIMUM (0)
        0x25,   0x01,                    //     LOGICAL_MAXIMUM (1)
        0x95,   0x10,                    //     REPORT_COUNT (16)
        0x75,   0x01,                    //     REPORT_SIZE (1)

        0x81,   0x02,                    //     INPUT (Data,Var,Abs)
//-----------------

        0xC0,                            //   END_COLLECTION

        0xc0
    };
#endif

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

#ifdef KEYB    
    p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);
#endif

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
#ifdef KEYB    
    hids_init_obj.is_kb                          = true;
#else    
    hids_init_obj.is_kb                          = false;
#endif    
    hids_init_obj.is_mouse                       = false;
    hids_init_obj.inp_rep_count                  = 1;
    hids_init_obj.p_inp_rep_array                = input_report_array;
#ifdef KEYB    
    hids_init_obj.outp_rep_count                 = 1;
    hids_init_obj.p_outp_rep_array               = output_report_array;
#else
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
#endif    
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    dis_init();
    bas_init();
    hids_init();
}


#ifdef KEYB

/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}
#endif

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    MY_APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    MY_APP_ERROR_CHECK(err_code);
}


#ifdef KEYB

/** @brief   Function for checking if the Shift key is pressed.
 *
 *  @returns true if the SHIFT_BUTTON is pressed. false otherwise.
 */
static bool is_shift_key_pressed(void)
{
    bool result;
    uint32_t err_code = bsp_button_is_pressed(SHIFT_BUTTON_ID,&result);
    MY_APP_ERROR_CHECK(err_code);
    return result;
}

/**@brief   Function for transmitting a key scan Press & Release Notification.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_instance     Identifies the service for which Key Notifications are requested.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern. 0 < pattern_len < 7.
 * @param[in]  pattern_offset Offset applied to Key Pattern for transmission.
 * @param[out] actual_len     Provides actual length of Key Pattern transmitted, making buffering of
 *                            rest possible if needed.
 * @return     NRF_SUCCESS on success, BLE_ERROR_NO_TX_BUFFERS in case transmission could not be
 *             completed due to lack of transmission buffer or other error codes indicating reason
 *             for failure.
 *
 * @note       In case of BLE_ERROR_NO_TX_BUFFERS, remaining pattern that could not be transmitted
 *             can be enqueued \ref buffer_enqueue function.
 *             In case a pattern of 'cofFEe' is the p_key_pattern, with pattern_len as 6 and
 *             pattern_offset as 0, the notifications as observed on the peer side would be
 *             1>    'c', 'o', 'f', 'F', 'E', 'e'
 *             2>    -  , 'o', 'f', 'F', 'E', 'e'
 *             3>    -  ,   -, 'f', 'F', 'E', 'e'
 *             4>    -  ,   -,   -, 'F', 'E', 'e'
 *             5>    -  ,   -,   -,   -, 'E', 'e'
 *             6>    -  ,   -,   -,   -,   -, 'e'
 *             7>    -  ,   -,   -,   -,   -,  -
 *             Here, '-' refers to release, 'c' refers to the key character being transmitted.
 *             Therefore 7 notifications will be sent.
 *             In case an offset of 4 was provided, the pattern notifications sent will be from 5-7
 *             will be transmitted.
 */
static uint32_t send_key_scan_press_release(ble_hids_t *   p_hids,
                                            uint8_t *      p_key_pattern,
                                            uint16_t       pattern_len,
                                            uint16_t       pattern_offset,
                                            uint16_t *     p_actual_len)
{
    uint32_t err_code;
    uint16_t offset;
    uint16_t data_len;
    uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];
    
    // HID Report Descriptor enumerates an array of size 6, the pattern hence shall not be any
    // longer than this.
    STATIC_ASSERT((INPUT_REPORT_KEYS_MAX_LEN - 2) == 6);

    ASSERT(pattern_len <= (INPUT_REPORT_KEYS_MAX_LEN - 2));

    offset   = pattern_offset;
    data_len = pattern_len;

    do
    {
        // Reset the data buffer. 
        memset(data, 0, sizeof(data));
        
        // Copy the scan code.
        memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);
        
        if (is_shift_key_pressed())
        {
            data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
        }

        if (!m_in_boot_mode)
        {
            err_code = ble_hids_inp_rep_send(p_hids, 
                                             INPUT_REPORT_KEYS_INDEX,
                                             INPUT_REPORT_KEYS_MAX_LEN,
                                             data);
        }
        else
        {
            err_code = ble_hids_boot_kb_inp_rep_send(p_hids,
                                                     INPUT_REPORT_KEYS_MAX_LEN,
                                                     data);
        }
        
        if (err_code != NRF_SUCCESS)
        {
            break;
        }
        
        offset++;
    } while (offset <= data_len);

    *p_actual_len = offset;

    return err_code;
}


/**@brief   Function for initializing the buffer queue used to key events that could not be
 *          transmitted
 *
 * @warning This handler is an example only. You need to analyze how you wish to buffer or buffer at
 *          all.
 *
 * @note    In case of HID keyboard, a temporary buffering could be employed to handle scenarios
 *          where encryption is not yet enabled or there was a momentary link loss or there were no
 *          Transmit buffers.
 */
static void buffer_init(void)
{
    uint32_t buffer_count;

    BUFFER_LIST_INIT();

    for (buffer_count = 0; buffer_count < MAX_BUFFER_ENTRIES; buffer_count++)
    {
        BUFFER_ELEMENT_INIT(buffer_count);
    }
}


/**@brief Function for enqueuing key scan patterns that could not be transmitted either completely
 *        or partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_hids         Identifies the service for which Key Notifications are buffered.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern.
 * @param[in]  offset         Offset applied to Key Pattern when requesting a transmission on
 *                            dequeue, @ref buffer_dequeue.
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
static uint32_t buffer_enqueue(ble_hids_t *            p_hids,
                               uint8_t *               p_key_pattern,
                               uint16_t                pattern_len,
                               uint16_t                offset)
{
    buffer_entry_t * element;
    uint32_t         err_code = NRF_SUCCESS;

    if (BUFFER_LIST_FULL())
    {
        // Element cannot be buffered.
        err_code = NRF_ERROR_NO_MEM;
    }
    else
    {
        // Make entry of buffer element and copy data.
        element                 = &buffer_list.buffer[(buffer_list.wp)];
        element->p_instance     = p_hids;
        element->p_data         = p_key_pattern;
        element->data_offset    = offset;
        element->data_len       = pattern_len;

        buffer_list.count++;
        buffer_list.wp++;

        if (buffer_list.wp == MAX_BUFFER_ENTRIES)
        {
            buffer_list.wp = 0;
        }
    }

    return err_code;
}


/**@brief   Function to dequeue key scan patterns that could not be transmitted either completely of
 *          partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  tx_flag   Indicative of whether the dequeue should result in transmission or not.
 * @note       A typical example when all keys are dequeued with transmission is when link is
 *             disconnected.
 *
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
static uint32_t buffer_dequeue(bool tx_flag)
{
    buffer_entry_t * p_element;
    uint32_t         err_code = NRF_SUCCESS;
    uint16_t         actual_len;

    if (BUFFER_LIST_EMPTY()) 
    {
        err_code = NRF_ERROR_NOT_FOUND;
    }
    else
    {
        bool remove_element = true;

        p_element = &buffer_list.buffer[(buffer_list.rp)];

        if (tx_flag)
        {
            err_code = send_key_scan_press_release(p_element->p_instance,
                                                   p_element->p_data,
                                                   p_element->data_len,
                                                   p_element->data_offset,
                                                   &actual_len);
            // An additional notification is needed for release of all keys, therefore check
            // is for actual_len <= element->data_len and not actual_len < element->data_len
            if ((err_code == BLE_ERROR_NO_TX_BUFFERS) && (actual_len <= p_element->data_len))
            {
                // Transmission could not be completed, do not remove the entry, adjust next data to
                // be transmitted
                p_element->data_offset = actual_len;
                remove_element         = false;
            }
        }

        if (remove_element)
        {
            BUFFER_ELEMENT_INIT(buffer_list.rp);

            buffer_list.rp++;
            buffer_list.count--;

            if (buffer_list.rp == MAX_BUFFER_ENTRIES)
            {
                buffer_list.rp = 0;
            }
        }
    }

    return err_code;
}


/**@brief Function for sending sample key presses to the peer.
 *
 * @param[in]   key_pattern_len   Pattern length.
 * @param[in]   p_key_pattern     Pattern to be sent.
 */
static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)
{
    uint32_t err_code;
    uint16_t actual_len;

    err_code = send_key_scan_press_release(&m_hids,
                                           p_key_pattern,
                                           key_pattern_len,
                                           0,
                                           &actual_len);
    // An additional notification is needed for release of all keys, therefore check
    // is for actual_len <= key_pattern_len and not actual_len < key_pattern_len.
    if ((err_code == BLE_ERROR_NO_TX_BUFFERS) && (actual_len <= key_pattern_len))
    {
        // Buffer enqueue routine return value is not intentionally checked.
        // Rationale: Its better to have a a few keys missing than have a system
        // reset. Recommendation is to work out most optimal value for
        // MAX_BUFFER_ENTRIES to minimize chances of buffer queue full condition
        UNUSED_VARIABLE(buffer_enqueue(&m_hids, p_key_pattern, key_pattern_len, actual_len));
    }


    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

#else

/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(ble_hids_evt_t *p_evt)
{
#ifdef USE_UART
    printf("on_hid_rep_char_write: ");
#endif    

    if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT)
    {
        uint32_t err_code;
        uint8_t  report_val;
        uint8_t  report_index = p_evt->params.char_write.char_id.rep_index;

        if (report_index == OUTPUT_REPORT_INDEX)
        {
            // This code assumes that the outptu report is one byte long. Hence the following
            // static assert is made.
            STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);

            err_code = ble_hids_outp_rep_get(&m_hids,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             &report_val);
            MY_APP_ERROR_CHECK(err_code);
#ifdef KEYB
            if (!m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) != 0))
            {
                // Caps Lock is turned On.
                err_code = bsp_indication_set(BSP_INDICATE_ALERT_3);
                MY_APP_ERROR_CHECK(err_code);

                keys_send(sizeof(m_caps_on_key_scan_str), m_caps_on_key_scan_str);
                m_caps_on = true;
            }
            else if (m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) == 0))
            {
                // Caps Lock is turned Off .
                err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
                MY_APP_ERROR_CHECK(err_code);

                keys_send(sizeof(m_caps_off_key_scan_str), m_caps_off_key_scan_str);
                m_caps_on = false;
            }
            else
            {
                // The report received is not supported by this application. Do nothing.
            }
#endif            
        }
    }
}

typedef struct 
{
    int8_t x,y,z,rx;
    uint16_t buttons; 
} joystick_data_t;


static void send_gamepad()
{
    static joystick_data_t old_data;
    static joystick_data_t data;
    uint8_t btns;
    
    data.x = (int8_t)(nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_3)-127);
    data.y = (int8_t)(nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_2)-127);
    data.z = (int8_t)(nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_4)-127);
    data.rx = (int8_t)(nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_5)-127);

    btns = buttons_read();

    data.buttons = (uint16_t)btns;

    if (abs(data.x-old_data.x)<2   &&
        abs(data.y-old_data.y)<2   &&
        abs(data.z-old_data.z)<2   &&
        abs(data.rx-old_data.rx)<2 &&
        data.buttons == old_data.buttons)
    {
        check_gamepad = true;
        return;
    }

    old_data = data;

    uint16_t r = 0;
    r |= ((btns>>0)&1)<<0;  // a
    r |= ((btns>>2)&1)<<1;  // b
    r |= ((btns>>1)&1)<<3;  // x
    r |= ((btns>>3)&1)<<4;  // y
    r |= ((btns>>4)&1)<<7;  // r
    r |= ((btns>>5)&1)<<6;  // l
    r |= ((btns>>6)&1)<<10;  // sel
    r |= ((btns>>7)&1)<<11;  // start

    data.buttons = r;

    uint32_t err_code = ble_hids_inp_rep_send(&m_hids,
                                                INPUT_REPORT_KEYS_INDEX,
                                                sizeof(joystick_data_t),
                                                (uint8_t*)&data);

    if (err_code != NRF_SUCCESS)
    {
        memset(&old_data, 0xff, sizeof(joystick_data_t));
#ifdef USE_UART
        printf("err: %0x08\n", (unsigned int)err_code);
#endif
    }
    else
    {     
#ifdef USE_UART
        //printf("1");
#endif
        //check_gamepad = false;
    }
}

#endif

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    MY_APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    MY_APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t *p_evt)
{
#ifdef USE_UART
    printf("on_hids_evt: ");
#endif    
    switch (p_evt->evt_type)
    {
#ifdef KEYB        
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;
#endif
        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            on_hid_rep_char_write(p_evt);
            break;

        case BLE_HIDS_EVT_NOTIF_DISABLED:
        {
#ifdef USE_UART
            printf("Disabled\n");
#endif      
            
            gamepad_timer_stop();      
            break;
        }

        case BLE_HIDS_EVT_NOTIF_ENABLED:
        {
#ifdef USE_UART
            printf("Enabled\n");
#endif      
            gamepad_timer_start();      

            dm_service_context_t   service_context;
            service_context.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
            service_context.context_data.len = 0;
            service_context.context_data.p_data = NULL;

            if (m_in_boot_mode)
            {
                // Protocol mode is Boot Protocol mode.
                if (
                    p_evt->params.notification.char_id.uuid
                    ==
                    BLE_UUID_BOOT_KEYBOARD_INPUT_REPORT_CHAR
                )
                {
                    // The notification of boot keyboard input report has been enabled.
                    // Save the system attribute (CCCD) information into the flash.
                    uint32_t err_code;

                    err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        MY_APP_ERROR_CHECK(err_code);
                    }
                    else
                    {
                        // The system attributes could not be written to the flash because
                        // the connected central is not a new central. The system attributes
                        // will only be written to flash only when disconnected from this central.
                        // Do nothing now.
                    }
                }
                else
                {
                    // Do nothing.
                }
            }
            else if (p_evt->params.notification.char_id.rep_type == BLE_HIDS_REP_TYPE_INPUT)
            {
                // The protocol mode is Report Protocol mode. And the CCCD for the input report
                // is changed. It is now time to store all the CCCD information (system
                // attributes) into the flash.
                uint32_t err_code;

                err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    MY_APP_ERROR_CHECK(err_code);
                }
                else
                {
                    // The system attributes could not be written to the flash because
                    // the connected central is not a new central. The system attributes
                    // will only be written to flash only when disconnected from this central.
                    // Do nothing now.
                }
            }
            else
            {
                // The notification of the report that was enabled by the central is not interesting
                // to this application. So do nothing.
            }
            break;
        }

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

#ifdef USE_UART
            printf("on_adv_evt :");
#endif            


    switch (ble_adv_evt)
    {
        /*
        case BLE_ADV_EVT_DIRECTED:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            MY_APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            MY_APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_SLOW:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            MY_APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_FAST_WHITELIST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            MY_APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_SLOW_WHITELIST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            MY_APP_ERROR_CHECK(err_code);
            break;
        */
        case BLE_ADV_EVT_IDLE:
#ifdef USE_UART
            printf("BLE_ADV_EVT_IDLE\n");
#endif
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
#ifdef USE_UART
            printf("BLE_ADV_EVT_WHITELIST_REQUEST\n");
#endif
            ble_gap_whitelist_t whitelist;
            ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;

            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            MY_APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_whitelist_reply(&whitelist);
            MY_APP_ERROR_CHECK(err_code);
            break;
        }
        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
#ifdef USE_UART
            printf("BLE_ADV_EVT_PEER_ADDR_REQUEST\n");
#endif
            ble_gap_addr_t peer_address;

            // Only Give peer address if we have a handle to the bonded peer.
            if(m_bonded_peer_handle.appl_id != DM_INVALID_ID)
            {
                            
                err_code = dm_peer_addr_get(&m_bonded_peer_handle, &peer_address);
                MY_APP_ERROR_CHECK(err_code);

                err_code = ble_advertising_peer_addr_reply(&peer_address);
                MY_APP_ERROR_CHECK(err_code);
                
            }
            break;
        }
        default:
#ifdef USE_UART
            printf("Unknown 0x%08x\n", ble_adv_evt);
#endif
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

#ifdef USE_UART
            printf("on_ble_evt :");
#endif            

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
#ifdef KEYB
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            MY_APP_ERROR_CHECK(err_code);
#endif
            m_conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;

            //timers_start();
#ifdef USE_UART
            printf("BLE_GAP_EVT_CONNECTED\n");
#endif            
            break;
        }   
        case BLE_EVT_TX_COMPLETE:
#ifdef USE_UART
            printf("BLE_EVT_TX_COMPLETE\n");
#endif 


#ifdef KEYB
            // Send next key event
            (void) buffer_dequeue(true);
#endif
            //send_gamepad();
        break;

        case BLE_GAP_EVT_DISCONNECTED:
            // Dequeue all keys without transmission.
            //timers_stop();
#ifdef USE_UART
            printf("BLE_GAP_EVT_DISCONNECTED\n");
#endif
            gamepad_timer_stop();      

            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            MY_APP_ERROR_CHECK(err_code);

            err_code = app_timer_stop(m_gamepad_polling_timer_id);
            MY_APP_ERROR_CHECK(err_code);    


#ifdef KEYB
            (void) buffer_dequeue(false);
#endif

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Reset m_caps_on variable. Upon reconnect, the HID host will re-send the Output 
            // report containing the Caps lock state.
#ifdef KEYB
            m_caps_on = false;
            // disabling alert 3. signal - used for capslock ON
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
            MY_APP_ERROR_CHECK(err_code);
#endif            
            break;

        case BLE_EVT_USER_MEM_REQUEST:
#ifdef USE_UART
            printf("BLE_EVT_USER_MEM_REQUEST\n");
#endif
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            MY_APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
#ifdef USE_UART
            printf("BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST\n");
#endif


            if(p_ble_evt->evt.gatts_evt.params.authorize_request.type
               != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_PREP_WRITE_REQ)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
                    || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op
                     == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type
                        == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle,&auth_reply);
                    MY_APP_ERROR_CHECK(err_code);
                }
            }
            break;
        case BLE_GAP_EVT_CONN_SEC_UPDATE:
        {
#ifdef USE_UART
            printf("BLE_GAP_EVT_CONN_SEC_UPDATE\n");
#endif
            uint16_t cccd_value;
            // Pupulate ble_gatts_value_t structure to hold received data and metadata.
            ble_gatts_value_t cccd_data;
            cccd_data.len = 2;
            cccd_data.offset = 0;
            cccd_data.p_value = (uint8_t*)&cccd_value;
            sd_ble_gatts_value_get(m_hids.conn_handle, m_hids.inp_rep_array[0].char_handles.cccd_handle, &cccd_data);
#ifdef USE_UART
            printf("cccd value m_hids.inp_rep_array: %d\n", cccd_value);
#endif
            if (cccd_value==1)
            {
                gamepad_timer_start();                      
            }
        }
        break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
#ifdef USE_UART
            printf("BLE_GATTS_EVT_TIMEOUT\n");
#endif
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            MY_APP_ERROR_CHECK(err_code);
            break;
        default:
#ifdef USE_UART
            printf("Unknown 0x%08x\n",p_ble_evt->header.evt_id);
#endif
            break;
    }
}


/**@brief   Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_hids_on_ble_evt(&m_hids, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
}


/**@brief   Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_APPSH_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, true);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    MY_APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    MY_APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
#ifdef KEYB
    static uint8_t * p_key = m_sample_key_press_scan_str;
    static uint8_t size = 0;
#endif
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                MY_APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                MY_APP_ERROR_CHECK(err_code);
            }
            break;
#ifdef KEYB
        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                keys_send(1, p_key);
                p_key++;
                size++;
                if (size == MAX_KEYS_IN_ONE_REPORT)
                {
                    p_key = m_sample_key_press_scan_str;
                    size = 0;
                }
            }
            break;
#endif

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t       err_code;
    uint8_t        adv_flags;
    ble_advdata_t  advdata;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    adv_flags                       = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = adv_flags;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options =
    {
        BLE_ADV_WHITELIST_ENABLED,
        BLE_ADV_DIRECTED_ENABLED,
        BLE_ADV_DIRECTED_SLOW_DISABLED, 0,0,
        BLE_ADV_FAST_ENABLED, APP_ADV_FAST_INTERVAL, APP_ADV_FAST_TIMEOUT,
        BLE_ADV_SLOW_ENABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT
    };

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, ble_advertising_error_handler);
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           ret_code_t           event_result)
{
    MY_APP_ERROR_CHECK(event_result);

    switch(p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_SECURITY_SETUP_COMPLETE:
            m_bonded_peer_handle = (*p_handle);
            break;
    }

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t  register_param;

    // Initialize peer device handle.
    err_code = dm_handle_initialize(&m_bonded_peer_handle);
    MY_APP_ERROR_CHECK(err_code);
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    MY_APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    MY_APP_ERROR_CHECK(err_code);
    
    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init( BSP_INIT_NONE, //BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    MY_APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    MY_APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    MY_APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    uint32_t err_code;

#ifdef USE_UART
    uart_init();
    printf("excelente\n");
#endif    

    configure_buttons();
    nrf_gpio_cfg_output(28);
    nrf_gpio_pin_write(28, 1);
    adc_config();

    // Initialize.
    app_trace_init();
    timers_init();
    buttons_leds_init(&erase_bonds);

    erase_bonds = ((buttons_read()>>7)&1) == 1; //start pressed?

#ifdef USE_UART    
    if (erase_bonds)
        printf("erasing bonds\n");
#endif
    ble_stack_init();
    scheduler_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
#ifdef KEYB    
    sensor_simulator_init();
#endif    
    conn_params_init();
#ifdef KEYB
    buffer_init();
#endif
    // Start execution.
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    MY_APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }    
}

/*
export PATH="/home/raul/repos/nrf51822/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH" 


arm-none-eabi-gdb  -batch \
  -ex "tar ext 192.168.1.3:2022" \
  -ex "mon s" \
  -ex "att 1" \
  -ex "file ../nrf51822/nordic_sdk/examples/ble_peripheral/ble_app_hids_keyboard/pca10028/s110/armgcc/_build/nrf51422_xxab_s110.out" \
  -ex "load ../nrf51822/nordic_sdk/examples/ble_peripheral/ble_app_hids_keyboard/pca10028/s110/armgcc/_build/nrf51422_xxab_s110.hex" \
  -ex continue 
#\
*/ 

