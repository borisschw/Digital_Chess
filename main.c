
// -----------------DCB Digital Chess Board ,all code functions are in this file ------------------
// 2019_09_08_V00  first version for demo                                                         //
// 2019_09_08_V01  fix ADC to none block mode                                                     //
//                 fixed bugs in the new_state decleration                                        //
// 2019_09_08_V02  fix ADC to block mode                                                          //
// 2019_12_08_V03  fix algo and add End of move                                                   //
//                  incress delay to the sample (10ms)                                            //
// 2019_20_09_V06   fix algo and cassteling                                                       //
//                  reduce sample time to (4.5ms)   for fast detection                            //
//                  NOTE  it is in Dbug mode only                                                 //
// 2019_25_09_V06   fixed hopfully the memory leakage                                             //
// 2019_26_09_V07   cstaling bug at sliding                                                       //
// 2019_26_09_V08   fixed cstaling bug at sliding                                                 //
//                  added a new state to the Move_algo, to filter part placement debounce         //
//                  chnaged the fast and slow detection mechanisem                                //
// 2019_12_10_V09   fixed balck casteling end of move bug                                         //
// 2019_12_10_V10   add pawn at the last line                                                     //
///

/// @param Debug_off_BLE_on debug mode on/off
#define Debug_off_BLE_on 1

#define PCB_VER 1

#include <stdint.h>
#include <string.h>
//#include "dcb.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

/*Includes for ADC*/
//#include "nrfx_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

///Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_CALIBRATION_INTERVAL 5

#define SAADC_SAMPLES_IN_BUFFER   2               //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 1                        //Set to 1 to enable BURST mode, otherwise set to 0.
/**/
/*Includes for BLE*/
#include "nordic_common.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
/**/

/*Includes for GPIO*/
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
/**/


/**/
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /*< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Chess Board DCB"                           /*< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /*< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /*< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /*< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define PAWNS_B 	0x01
#define ROOK_B 		0x02
#define KNIGHT_B 	0x03
#define BISHOP_B 	0x04
#define QUEEN_B 	0x05
#define KING_B 		0x06

#define PAWNS_W   0x11
#define ROOK_W 		0x12
#define KNIGHT_W 	0x13
#define BISHOP_W 	0x14
#define QUEEN_W   0x15
#define KING_W 		0x16

#define SW_BLACK 7
#define SW_WHITE 8



/*Module Pin defines*/
#define BLOCK_ADDR0 13
#define BLOCK_ADDR1 10
#define BLOCK_ADDR2 9

#define ADC_BAT 3
#define ADC_PD0 2   // an0
#define ADC_PD1 4   // an1

#define ADDR0 16
#define ADDR1 17
#define ANALOG_SELECT0  23
#define ANALOG_SELECT1  24
#define EN_PD 31
#define EN_AMP 29
#define EN_BLOCK 30
#define EN_SYS 20

#define TP1 19
#define TP2 5
#define TP5 28
#define TP6 14
#define TP7 15

#define INIT_CHANGE_VALUE 0xFF
#define MAX_NUMBER_OF_CHANGES 8
#define BLOCK_NUM 8
#define NUM_OF_CELLS 64
#define BITS_IN_BYTE 8
#define BIT(n) (0x1U <<(n))
#define NBIT(n) (0xFF - (0x1U <<(n)))

#define PHOTO_DIODE_DELAY_US 30
#define ANALOG_MUX_DELAY_US 5
#define DEBOUNCE_VAL_slow 10
#define DEBOUNCE_VAL_fast 0
#define EOM_THRESHOLD 100 // 150*3mSec =750msec

#define HIGH_ADC_THRESHOLD 7
#define LOW_ADC_THRESHOLD 4

/**
 * @brief commands to and from Android App
 *
 */
enum command_value{
/*-------To Android----------------*/
ping_from_dcb   = 0x01,
mask_bit_array  = 0x02,
full_bit_array  = 0x03,
end_of_move     = 0x04,

/*-------from android--------------*/
ping_from_android = 0x80,
init_state        = 0x81,
start_of_game     = 0x82,
end_of_game       = 0x83,
resume_game       = 0x84,
header            = 0xAA
};

/**
 * @brief move_algo state number enum
 *
 */
enum move_algo_case{
start_move        =1,
move_phase_1      =2,
move_phase_2      =3,
capture_phase     =4,
castling_phase_1  =5,
castling_phase_2  =6,
end_move_phase_1  =7,
end_move_phase_2  =8
};
/**< BLE NUS service instance. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
/**< GATT module instance. */
NRF_BLE_GATT_DEF(m_gatt);
/**< Context for the Queued Write module.*/
NRF_BLE_QWR_DEF(m_qwr);
/**< Advertising module instance. */
BLE_ADVERTISING_DEF(m_advertising);
void saadc_sampling_event_init(void);


/*-------------------------------------Function decalraitions--------------------------------------------------------*/
static void send_err(uint8_t err_type,uint8_t err_pos);
static int set_block(uint8_t block_num);
static int set_addr(uint8_t addr_num);
static int set_analog_mux(uint8_t ana_num);
static int read_adc(_Bool *data);
static int init_dcb();
static int move_algo(uint8_t change_val, _Bool reset);
static int enable_pd_block(uint8_t block_num);
static int read_block();
static int read_board_position();
static int compare_block();//(uint8_t *current_state, uint8_t *new_state);
int compare_arrays(uint8_t a[],uint8_t b[],uint8_t size);

static int send_board(void);
static int send_recovery_board(void);


static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

//typedef struct Data_packet {
//  uint8_t header;
//  uint8_t command;
//  uint8_t length;
//  uint8_t data[64];
//  uint8_t checksum;
//} Data_packet;

volatile uint8_t data_packet[256];
//uint8_t* data_to_send;


static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_calibrate = false;


/*---------------------------------- Global Variables--------------------------------------------------------------*/

/**
 *
 * change array [change number][0:type of change; 1:number of times the change is persistant]
 */
volatile uint8_t change_array[MAX_NUMBER_OF_CHANGES][2];

/*This array updated on every player switch
and saves the current_position*/
uint8_t recovery_current_state[8];
/**
 * Holds the current board state every bit is place on the board
 */
volatile uint8_t current_state[8];

uint8_t new_state[8];// Holds the new board state

 /** Holds the current board state with the figures names,
  * each byte is figure description
  */
volatile uint8_t current_state_board[64] = {0};
/**This array updated on every player switch
 * and saves the current_state_board*/
volatile uint8_t recovery_current_state_board[64] = {0};


volatile int global_index;
volatile int odd_array[32];
volatile int even_array[32];
void send_end_of_move(_Bool white);
nrf_saadc_value_t ADC_result[2];
volatile _Bool ADC_Val[2];
volatile uint8_t move_case;
volatile uint8_t position_eror;
volatile uint8_t DEBOUNCE_VAL;
volatile uint8_t pause_game_flag;

volatile uint8_t line_debug;
//volatile bool sampling_flag = false;


volatile uint16_t sampling_data;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

volatile _Bool recieved_data_flag = false;
volatile _Bool white_turn = true;
volatile _Bool Flag_EOM_timer = false;
volatile _Bool start_of_game_flag = false;
volatile _Bool SOG_update_flag = true;
volatile _Bool flag_castling;
volatile uint32_t adc_treshold = 7;

/*Indicated that end of move command should be sent*/
volatile _Bool send_EOM_flag = false;



volatile uint16_t EOM_counter = 0;

/**
 * @brief Holds the initial board state
 *
 */
uint8_t board_state_array[64] = {
  ROOK_W, KNIGHT_W, BISHOP_W, QUEEN_W, KING_W, BISHOP_W, KNIGHT_W, ROOK_W,
  PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W,
  0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,
  PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B,
  ROOK_B,KNIGHT_B,BISHOP_B,QUEEN_B,KING_B, BISHOP_B, KNIGHT_B, ROOK_B};

/**
 * @brief
 */
uint8_t board_lookup[32] = { 0,   //[0,0,0,0,0,0,0,0], 0
                            4,    //[0,0,0,0,0,1,0,0],  1
                            8,    //[0,0,0,0,1,0,0,0],  2
                            12,   //[0,0,0,0,1,1,0,0],  3
                            66,   //[0,1,0,0,0,0,1,0],  4
                            70,   //[0,1,0,0,0,1,1,0],
                            74,   //[0,1,0,0,1,0,1,0],
                            78,   //[0,1,0,0,1,1,1,0],
                            16,   //[0,0,0,1,0,0,0,0],  8
                            20,   //[0,0,0,1,0,1,0,0],  9
                            24,   //[0,0,0,1,1,0,0,0],
                            28,   //[0,0,0,1,1,1,0,0],
                            82,   //[0,1,0,1,0,0,1,0],
                            86,   //[0,1,0,1,0,1,1,0],
                            90,   //[0,1,0,1,1,0,1,0],
                            94,   //[0,1,0,1,1,1,1,0],
                            33,   //[0,0,1,0,0,0,0,1], 16
                            37,   //[0,0,1,0,0,1,0,1], 17
                            41,   //[0,0,1,0,1,0,0,1],
                            45,   //[0,0,1,0,1,1,0,1],
                            99,   //[0,1,1,0,0,0,1,1],
                            103,  //[0,1,1,0,0,1,1,1],
                            107,  //[0,1,1,0,1,0,1,1],
                            111,  //[0,1,1,0,1,1,1,1],
                            49,   //[0,0,1,1,0,0,0,1], 24 // fxed value 10.9.2019
                            53,   //[0,0,1,1,0,1,0,1],
                            57,   //[0,0,1,1,1,0,0,1],
                            61,   //[0,0,1,1,1,1,0,1],
                            115,  //[0,1,1,1,0,0,1,1],
                            119,  //[0,1,1,1,0,1,1,1],
                            123,  //[0,1,1,1,1,0,1,1],
                            127   //[0,1,1,1,1,1,1,1]
                            };

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint8_t i;
    uint8_t b,a =2;

    //uint8_t *temp_data;
    uint8_t calc_checksum;

    //data_packet = (uint8_t*)malloc(sizeof(uint8_t) * p_evt->params.rx_data.length);

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                //err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                data_packet[i] = p_evt->params.rx_data.p_data[i];

                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        recieved_data_flag = true;

    }
    //free(temp_data);

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
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
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
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

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**
 * @brief Function for handling events from the GATT library.
 *
 * @param p_gatt
 * @param p_evt
 */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    //static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t data_array[256]; // change to 256
    static uint8_t index = 0;
    uint32_t       err_code;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
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
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
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


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------saadc_callback------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief saadc_callback
 *
 * @param p_event
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    NRF_LOG_INFO("ADC event");
}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------saadc_init-------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief initiate ADC
 *
 */
void saadc_init(void)
{
    uint32_t err_code;
    nrf_drv_saadc_config_t saadc_config;

    //Configure SAADC

    nrf_saadc_channel_config_t channel_config0 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    #ifdef PCB_VER
      nrf_saadc_channel_config_t channel_config1 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    #else
      nrf_saadc_channel_config_t channel_config1 = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    #endif
    saadc_config.low_power_mode = true;                                     //Enable low power mode.
    saadc_config.resolution         = NRF_SAADC_RESOLUTION_10BIT;           //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample         = 0;        //disable.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                 //Set SAADC interrupt to low priority.

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);
    channel_config0.gain       = NRF_SAADC_GAIN1_4;
    //channel_config0.acq_time   = NRF_SAADC_ACQTIME_5US;

    channel_config1.gain       = NRF_SAADC_GAIN1_4;
    //channel_config1.acq_time   = NRF_SAADC_ACQTIME_5US;

     err_code = nrf_drv_saadc_channel_init (0, &channel_config0);
     APP_ERROR_CHECK(err_code);

     err_code = nrf_drv_saadc_channel_init (1, &channel_config1);
     APP_ERROR_CHECK(err_code);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------data_packet_cs---------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Compute data packet check sum
 *
 * @return uint8_t
 */
uint8_t data_packet_cs()//(Data_packet* data)
{
    uint8_t i = 0;
    uint8_t checksum;

    checksum = 0; // data_packet[] + data->command + data->length;

    /*length is onlt for data section , need to add 2 for header and command*/
    for (i = 0; i < data_packet[2] + 3; i++)
    {
      checksum = checksum + data_packet[i];
    }

    if (checksum == data_packet[i])
      return 0;
    else
      return checksum;

}

//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------send_ack---------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief send acknoladge to App
 *
 * @param command
 * @param ACK
 * @return uint32_t
 */
uint32_t send_ack(uint8_t command, _Bool ACK)
{
  //data_to_send = (uint8_t*) malloc(sizeof(uint8_t) * 6);
  uint8_t data_to_send[6] = {0};
  uint16_t length = 6;
  uint8_t recieved_checksum;
  uint32_t ret;
  uint8_t calc_checksum = 0;

  if (command == ping_from_android)
    recieved_checksum = data_packet[3];
  else
    recieved_checksum = data_packet[data_packet[2] + 3];


  data_to_send[0] = header;
  data_to_send[1] = command;
  data_to_send[2] = 2;

  if (ACK)
    data_to_send[3] = 0x0A;
  else
    data_to_send[3] = 0x0D;

  data_to_send[4] = recieved_checksum;

  for(int i = 0; i < 5; i++)
    calc_checksum += data_to_send[i];

  data_to_send[5] = calc_checksum;

  ret = ble_nus_data_send(&m_nus, data_to_send, &length, m_conn_handle);

  free(data_to_send);

  return ret;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------send_end_of_move-------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Sending end of move command to the App and saving the position to the recovery arrays.
 * @param recovery_current_state is the array that hold the current_position at the end of move. In case of fault move recover from it.
 * @param recovery_current_state_board is the array that hold the current_state_board at the end of move. In case of fault move recover from it.
 * @param white indicated the player that finish the move.
 */
void send_end_of_move(_Bool white)
{
  uint8_t data_to_send1[5] = {0};
  uint16_t length;
  uint8_t checksum = 0;
  uint8_t index =0;

  /*Save current state to recovery memory*/
  for (index = 0 ; index < 8 ; index++)
  {
    recovery_current_state[index] = current_state [index];
  }

  for (index = 0 ; index < 64 ; index++)
  {
    recovery_current_state_board[index] = current_state_board [index];
  }


//  memcpy(recovery_current_state, current_position,sizeof(current_position));
//  memcpy(recovery_current_state_board, current_state_board,sizeof(current_state_board));

  data_to_send1[0] = 0xAA;
  data_to_send1[1] = 0x04;
  data_to_send1[2] = 0x01;

  if (white)
    data_to_send1[3] = 0x10;
  else
    data_to_send1[3] = 0x01;

  for (int i = 0; i < 4 ; i++)
  {
    checksum += data_to_send1[i];
  }

  data_to_send1[4] = checksum;
  length = 5;

ble_nus_data_send(&m_nus, data_to_send1, &length, m_conn_handle);

}

/**
 * @brief Compares 2 arrays
 *
 * @param a
 * @param b
 * @param size
 * @return int
 */
int compare_arrays(uint8_t a[],uint8_t b[],uint8_t size)
{
  uint8_t index = 0;
  for(index = 0; index < size; index++)
  {
      if (a[index] != b[index])
        return 0;
  }
  return 1;
}



//---------------------------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------send_board------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Send the full current state of the board to the App
 *
 * @return int
 */
int send_board(void)
{
    uint8_t data_to_send1[68] = {0};
    uint16_t length;
    uint8_t checksum = 0;
   // int i;

    data_to_send1[0] = 0xAA;
    data_to_send1[1] = 0x03;
    data_to_send1[2] = 0x40;

    for (int i = 0; i < 64; i++ )
    {
      data_to_send1[i + 3] = current_state_board[i];
      //data_to_send1[i + 3] =board_state_array1[i];
    }

    for (int i = 0 ; i < 67 ; i++)
    {
       checksum += data_to_send1[i];
    }
    data_to_send1[67] = checksum;
    length = 68;

    ble_nus_data_send(&m_nus, data_to_send1, &length, m_conn_handle);

}



//---------------------------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------send_recovery_board------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Send the full recovery state of the board (the board at the start of the current move)
 *
 * @return int
 */
int send_recovery_board(void)
{
    uint8_t data_to_send1[68] = {0};
    uint16_t length;
    uint8_t checksum = 0;
   // int i;

    data_to_send1[0] = 0xAA;
    data_to_send1[1] = 0x03;
    data_to_send1[2] = 0x40;

    for (int i = 0; i < 64; i++ )
    {
      data_to_send1[i + 3] = recovery_current_state_board[i];
      //data_to_send1[i + 3] =board_state_array1[i];
    }

    for (int i = 0 ; i < 67 ; i++)
    {
       checksum += data_to_send1[i];
    }
    data_to_send1[67] = checksum;
    length = 68;

    ble_nus_data_send(&m_nus, data_to_send1, &length, m_conn_handle);

}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------send_current_position-------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//

// int send_current_position(void)
// {
//     uint8_t data_to_send1[12] = {0};
//     uint16_t length;
//     uint8_t checksum = 0;
//    // int i;

//     data_to_send1[0] = 0xAA;
//     data_to_send1[1] = 0x02;
//     data_to_send1[2] = 0x08;

//     for (int i = 0; i < 8; i++ )
//     {
//       data_to_send1[i + 3] = current_state[i];
//     }

//     for (int i = 0 ; i < 11 ; i++)
//     {
//        checksum += data_to_send1[i];
//     }
//     data_to_send1[12] = checksum;
//     length = 13;

//     ble_nus_data_send(&m_nus, data_to_send1, &length, m_conn_handle);

// }

//---------------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------send_err----------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Send error message to the App.
 *
 * @param err_type
 * @param err_pos
 */
void send_err(uint8_t err_type ,uint8_t err_pos)
{
    uint8_t data_to_send1[6] = {0};
    uint16_t length;
    uint8_t checksum = 0;

    data_to_send1[0] = 0xAA;
    data_to_send1[1] = 0x05;
    data_to_send1[2] = 0x02;
    data_to_send1[3] = err_type;
    data_to_send1[4] = err_pos;
    for (int i = 0 ; i < 5 ; i++)
    {
       checksum += data_to_send1[i];
    }
    data_to_send1[5] = checksum;
    length = 6;

    ble_nus_data_send(&m_nus, data_to_send1, &length, m_conn_handle);

}

//---------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------init_dcb-------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Initiate HW GPIO
 *
 * @return int
 */
int init_dcb()
{
  int i;
  /*Init GPIO*/
  nrf_gpio_cfg_output(BLOCK_ADDR0);
  nrf_gpio_cfg_output(BLOCK_ADDR1);
  nrf_gpio_cfg_output(BLOCK_ADDR2);

  nrf_gpio_cfg_output(ADDR0);
  nrf_gpio_cfg_output(ADDR1);

  nrf_gpio_cfg_output(ANALOG_SELECT0);
  nrf_gpio_cfg_output(ANALOG_SELECT1);


  nrf_gpio_cfg_input(ADC_BAT,NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(ADC_PD0,NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(ADC_PD1,NRF_GPIO_PIN_NOPULL);

  nrf_gpio_cfg_input(ADC_PD0,NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(ADC_PD1,NRF_GPIO_PIN_NOPULL);

  /*Those 2 inputs for changeing the Threshold of the ADC*/
  nrf_gpio_cfg_input(SW_BLACK,NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(SW_WHITE,NRF_GPIO_PIN_NOPULL);


  nrf_gpio_cfg_output(EN_PD);
  nrf_gpio_cfg_output(EN_AMP);
  nrf_gpio_cfg_output(EN_BLOCK);
  nrf_gpio_cfg_output(EN_SYS);
  nrf_gpio_cfg_output(TP1);
  nrf_gpio_cfg_output(TP2);
  nrf_gpio_cfg_output(TP5);
  nrf_gpio_cfg_output(TP6);
  nrf_gpio_cfg_output(TP7);
  read_board_position();


}

//----------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------set_addr------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Set address of the cell to read.
 *
 * @param addr_num
 * @return int
 */
int set_addr(uint8_t addr_num)
{
  if ((addr_num < 4) && (addr_num >= 0))
  {

    switch(addr_num)
    {
      case 0:
            nrf_gpio_pin_clear(ADDR0);
            nrf_gpio_pin_clear(ADDR1);
        break;
      case 1:
            //a=addr_num;
            //b=a;
            nrf_gpio_pin_set(ADDR0);
            nrf_gpio_pin_clear(ADDR1);
        break;
      case 2:
            nrf_gpio_pin_clear(ADDR0);
            nrf_gpio_pin_set(ADDR1);
        break;
      case 3:
            nrf_gpio_pin_set(ADDR0);
            nrf_gpio_pin_set(ADDR1);
        break;
    }
      return 0;
  }

    else
      return 1;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------set_analog_mux------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
int set_analog_mux(uint8_t ana_num)
{
  if ((ana_num < 4) && (ana_num >= 0))
  {
    //a= ana_num;
   // b = a;
    switch(ana_num)
    {
      case 0:
            nrf_gpio_pin_clear(ANALOG_SELECT0);
            nrf_gpio_pin_clear(ANALOG_SELECT1);
        break;
      case 1:

            nrf_gpio_pin_set(ANALOG_SELECT0);
            nrf_gpio_pin_clear(ANALOG_SELECT1);
        break;
      case 2:
            //a=ana_num;
          //  b=a;
            nrf_gpio_pin_clear(ANALOG_SELECT0);
            nrf_gpio_pin_set(ANALOG_SELECT1);
        break;
      case 3:
            nrf_gpio_pin_set(ANALOG_SELECT0);
            nrf_gpio_pin_set(ANALOG_SELECT1);
        break;
    }

     return 0;
  }

    else
      return 1;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------- set_block--------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Set the block
 * There are 8 block in each block 8 cells
 * @param block_num
 * @return int
 */
int set_block(uint8_t block_num)
{
  if ((block_num < 8) && (block_num >= 0))
  {
    switch(block_num)
    {
      case 0:
            nrf_gpio_pin_clear(BLOCK_ADDR0);
            nrf_gpio_pin_clear(BLOCK_ADDR1);
            nrf_gpio_pin_clear(BLOCK_ADDR2);
            break;
      case 1:
            nrf_gpio_pin_set(BLOCK_ADDR0);
            nrf_gpio_pin_clear(BLOCK_ADDR1);
            nrf_gpio_pin_clear(BLOCK_ADDR2);
            break;
      case 2:
            nrf_gpio_pin_clear(BLOCK_ADDR0);
            nrf_gpio_pin_set(BLOCK_ADDR1);
            nrf_gpio_pin_clear(BLOCK_ADDR2);
            break;
      case 3:
            nrf_gpio_pin_set(BLOCK_ADDR0);
            nrf_gpio_pin_set(BLOCK_ADDR1);
            nrf_gpio_pin_clear(BLOCK_ADDR2);

            break;
      case 4:
            nrf_gpio_pin_clear(BLOCK_ADDR0);
            nrf_gpio_pin_clear(BLOCK_ADDR1);
            nrf_gpio_pin_set(BLOCK_ADDR2);
            break;
      case 5:
            nrf_gpio_pin_set(BLOCK_ADDR0);
            nrf_gpio_pin_clear(BLOCK_ADDR1);
            nrf_gpio_pin_set(BLOCK_ADDR2);
            break;
      case 6:
            nrf_gpio_pin_clear(BLOCK_ADDR0);
            nrf_gpio_pin_set(BLOCK_ADDR1);
            nrf_gpio_pin_set(BLOCK_ADDR2);
            break;
      case 7:
            nrf_gpio_pin_set(BLOCK_ADDR0);
            nrf_gpio_pin_set(BLOCK_ADDR1);
            nrf_gpio_pin_set(BLOCK_ADDR2);
            break;

      }

      return 0;
   }
   else
      return 1;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------- read_board_position----------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Read the board position to new_state[8].
 * It sets the Muxes and buffers gpio
 * @warning Problem with function, whene reading once
 * @return int
 */
int read_board_position(void)
{
  static uint8_t index_B;
  int a,b;
  uint8_t line_val0;
  uint8_t line_val1;
  a = 1;


  nrf_gpio_pin_set(EN_SYS);
  nrf_gpio_pin_set(EN_PD);
  nrf_gpio_pin_set(EN_AMP);
  nrf_gpio_pin_set(EN_BLOCK);
  nrf_gpio_pin_set(TP5);
  nrf_delay_us(PHOTO_DIODE_DELAY_US);

  /* Scanning the board 32 times and each
  * time get 2 cells at a time
  */
  line_val0 =0;
  line_val1 =0;
  for (index_B = 0; index_B < NUM_OF_CELLS/2; index_B++)
  {
    global_index = index_B;
    //nrf_gpio_pin_set(TP2);

    /*This GPIO discharge the capacitor in the input of the opAMP(connects it to GND)*/
    nrf_gpio_pin_set(TP5);
    set_block(board_lookup[index_B] >> 4);
    set_addr((board_lookup[index_B] >> 2) & 3);
    set_analog_mux((board_lookup[index_B]) & 3);
    /*After setting the muxes, release the mosfet to allow measurments*/
    nrf_gpio_pin_clear(TP5);

    nrf_delay_us(ANALOG_MUX_DELAY_US);
    read_adc(&ADC_Val);    // added the & simbol due to compailer warning
    if (ADC_Val[0] == true)
      line_val0 = line_val0 | BIT(index_B % BITS_IN_BYTE);  // set bit

    if (ADC_Val[1] == true)
      line_val1 = line_val1 | BIT(index_B % BITS_IN_BYTE); // set bit

    if(((index_B+1) % BITS_IN_BYTE) == 0) // if scanned a line - update the array
    {
      new_state[((2 * index_B) / BITS_IN_BYTE)-1] = line_val0;
      new_state[((2 * index_B) / BITS_IN_BYTE)] = line_val1;
      line_val0 =0;
      line_val1 =0;
    }

  }

  //nrf_gpio_pin_clear(EN_PD);
  nrf_gpio_pin_clear(EN_BLOCK);
  nrf_gpio_pin_clear(EN_AMP);
  nrf_gpio_pin_clear(EN_SYS);
  set_block(board_lookup[0] >> 4);
  set_addr((board_lookup[0] >> 2) & 3);
  set_analog_mux((board_lookup[0]) & 3);

  return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------ compare_block -------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Find Changes in the board.
 * Compares the current_state[8] and new_state[8] and update change_array[8][3].
 *
 * @return int
 */
int compare_block(void)//(uint8_t *current_state, uint8_t *new_state)
{
  int i,j;
  volatile uint8_t line;
  volatile uint8_t column;
  volatile uint8_t current_bit;
  volatile uint8_t new_bit;

  /* Change_location:
   * Bit       7         |6    |5    |4    |3     |2    |1    |0
   * meaning : add/remove|line2|line1|line0|------|col2 |col1 |col0
  */
  volatile static uint8_t change_location;

  volatile static uint8_t change;
  volatile static _Bool flag_new_change;
  uint8_t ret_update;

  for (line = 0; line < 8; line++)
  {
    /*Check if new move happened on one of the lines*/
    if (current_state[line] != new_state[line])
    {
      for(column = 0; column < 8; column++)
      {
        current_bit = ((current_state[line] >> column) & 0x01) ;
        new_bit     = ((new_state[line]     >> column) & 0x01) ;
        if(current_bit != new_bit)
        {
          //change_val structure [1(added)/0(removed), col2,col1,col0,0, line2, line1, line0]

          //MSB nibble is the line number, LSB nibble is the column number
          change_location = (line<<4) + column ;

          /* A piece was placed on the squre; MSB bit is add/remove piece. */
          if (new_bit)
            change_location = change_location | 0x80;

          flag_new_change = true;
          for(change = 0; change < MAX_NUMBER_OF_CHANGES; change++)
          {
            if (change_array[change][0] == change_location)
            {
                /* ADD 2 in case of change and subtruct 1 from everybody later so
                 * eventually add 1 to the new changes.
                 */
                change_array [change][1] = change_array[change][1] + 2;
                flag_new_change = false;
                break; // ---- verify it isn't exiting the function but only the loop---
            }
          }
          /* if the change happned for the first time.
          *  set the counter (change_array[change][1]) to 0x01
          * */
          if (flag_new_change)
          {
            /*Find an empty space (which is still in initail state) to save the new change*/
            for(change = 0; change < MAX_NUMBER_OF_CHANGES; change++)
            {
              if (change_array[change][0] == INIT_CHANGE_VALUE)
              {
                  change_array[change][0] = change_location;
                  change_array[change][1] = 0x01;
                  break;
              }
            }
          }
        }
      }
    }
  }

  for(change = 0; change < MAX_NUMBER_OF_CHANGES; change++)
  {
    /*If there there is value in change_location, but the counter = 0 ->
    * -> something is not correct
    */
    if (change_array[change][0]!=INIT_CHANGE_VALUE)
    {
      if (change_array[change][1] == 0x00)
      {
        change_array[change][0] = INIT_CHANGE_VALUE;
        change_array[change][1] = INIT_CHANGE_VALUE;
      }
      else
      {
        /*Subtract 1 from the counter that was increased by 2 before*/
        --change_array[change][1];
      }
      /* Check if enough time passed (change_array[change][1] <=> kind of a timer)
      *  so the change in piece is stable
      */
      if ((change_array[change][1] > DEBOUNCE_VAL) && (change_array[change][1] != 0xFF ))
      {
        /*Check if the move is correct.
          1-> legal move
          0-> fault move
        */
        ret_update = move_algo(change_array[change][0] ,false);

        if (ret_update == 1) /*legal move*/
        {
          line   = (change_array[change][0]>>4) & 0x07;
          column = change_array[change][0] & 0x07;

          /*If the piece was set(grater than 0x79 means MSB == "1")
            Pay attention: the MSB bit of low nibble is not used and equal to 0
            that's why the 0x77...0x7F are impossible values.*/
          if (change_array[change][0]>0x79)
            current_state[line]= current_state[line] | BIT(column);
          else /*piece was removed*/
            current_state[line]= current_state[line] & NBIT(column);

          if (send_EOM_flag )
          {
            send_EOM_flag = false;
            send_end_of_move(white_turn);
          }

          change_array[change][0] = INIT_CHANGE_VALUE;
          change_array[change][1] = INIT_CHANGE_VALUE;
          //send_current_position();
          send_board();
        }
        else if (ret_update==0) /*Illigal move*/
        {
          /* Send recovery board*/
          send_recovery_board();

          nrf_delay_ms(10);

          /* This functions sends error message to ble app*/
          send_err(0,position_eror);

          /*Clear the state machine and flags as it will act as new move*/
          Flag_EOM_timer = false;
          flag_castling = false;
          move_case = start_move;
          pause_game_flag = true ;

          for(change = 0; change < MAX_NUMBER_OF_CHANGES; change++)
          {
              change_array[change][0] = INIT_CHANGE_VALUE;
              change_array[change][1] = INIT_CHANGE_VALUE;
          }
          break;
        }
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------Read ADC -------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//

int read_adc(_Bool *val)
{
    int temp;
    static uint8_t x_val,y_val;
    ret_code_t err_code;
    nrf_gpio_pin_clear(TP6);
    nrf_gpio_pin_set(TP2);
    // Sample channel 0
    err_code =nrfx_saadc_sample_convert(0, &ADC_result[0]);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_clear(TP2);
    //NRF_LOG_INFO("Channel 0 sample: %i", ADC_result[0]);
    nrf_gpio_pin_set(TP2);
    // Sample channel 1
    err_code =nrfx_saadc_sample_convert(1, &ADC_result[1]);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_clear(TP2);
    //NRF_LOG_INFO("Channel 1 sample: %i", ADC_result[1]);

    if( ADC_result[0] < 0)
        ADC_result[0] = 0;
    if( ADC_result[1] < 0)
        ADC_result[1] = 0;
        if (global_index>31)
          global_index =31;
    odd_array[global_index]  =ADC_result[0];
    even_array[global_index] =ADC_result[1];
//    if((global_index==12)&&(ADC_result[1]>10))
//    {
//       nrf_gpio_pin_set(TP6);
//    }
    if (ADC_result[0] < adc_treshold) // AN2 chanel X
        val[0] = true;
    else
        val[0] = false;


    if (ADC_result[1] < adc_treshold)//AN0 chanel Y
        val[1] = true;
    else
        val[1] = false;

    //sampling_flag = false;
    //--------------- Debug only ----------------------//
//    x_val = ADC_result[0]>>2;
//    app_uart_put(x_val);
//    nrf_delay_ms(1);
//    y_val = ADC_result[1]>>2;
//    nrf_delay_ms(1);
//    app_uart_put(y_val);

    //--------------- Debug only ----------------------//
   return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------- move_algo -------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//


/**
 * @brief This function will check if the move is legal.
 * It holds state machine which check every stet during the move,
 * and checks for castling operation.
 *
 * @param[in] change_val [number of change][0->change_location; 1-> debounce counter]
 * @param[in] reset will set the move to start_move and clear **type_and_place[3][2]** array
          global_index = flag_castling + move_case;
 * @attention global_index is just dummy variable (with no real purpose) for debugger not to ignore code sections.
 *
 *
 * @return 1 if success and 0 if falt move
 */
int move_algo(uint8_t change_val, _Bool reset)
{
  /* type_and_place[index 0: piece removed; index 1: piece added][index 0:place; index 1: type] */
  volatile static uint8_t type_and_place[3][2];

  volatile static uint8_t place;
  volatile static uint8_t piece_type;

  if (reset == true)
  {
    white_turn = true;
    move_case = start_move;
    for (int i=0; i<3; i++)
      for (int j=0; j<2; j++)
        type_and_place[i][j] = 0;
  }

  /* 0-63 posision and type*/
  place = ((change_val & 0x70) >> 1) + (change_val & 0x07);

  /* Get piece type.*/
  piece_type = current_state_board[place];

  /* In case error will happen. */
  position_eror = place;

  //reset_timer_EOM = True; // reset the counter for END of move

  switch (move_case)
  {
    case start_move://----------------------------------------------------------------------------------------------//
      type_and_place[0][0] = place;      //
      type_and_place[0][1] = piece_type; //
      flag_castling = false;

      /* MSB of change_val is the operation type and don't remove empty cell
         at first piece should be removed from the cell (That's how chess pieces move across the board) */
      if ((change_val < 0x79)&&(piece_type!=0)) /* if Remove piece */
      {
        /* See macros at top of the file for piece_type number.
        *  Above 0x10 are whites, below, blacks */
        if ((white_turn)&&(piece_type > 0x10))
        {
          /* Check for Castling */
          if ((piece_type == KING_W) && (place == 0x04) &&
              (((current_state[0] & 0x1F) == 0x11) || ((current_state[0] & 0xF0) == 0x90) ))
            flag_castling = true;

          move_case = move_phase_1;
          DEBOUNCE_VAL = DEBOUNCE_VAL_fast;
          global_index = flag_castling + move_case;//dummy variable
          current_state_board[place] = 0;
        }
        else if ((!white_turn)&&(piece_type < 0x10)) /*Same as white case*/
        {
          /* Check for Castling */
          if ((piece_type == KING_B) && (place == 0x3C)&&
              (((current_state[7] & 0x1F) == 0x11) || ((current_state[7] & 0xF0) == 0x90) ))
            flag_castling = true;

          move_case = move_phase_1;
          DEBOUNCE_VAL = DEBOUNCE_VAL_fast;
          global_index = move_case;//dummy variable
          current_state_board[place] = 0;
        }
        /*If player on his turn move opponent's piece and it's
        * the last piece the opponent moved
        */
        else if (type_and_place[1][0] == place)
        {
          move_case = end_move_phase_1;
        }
        else
          return 0;
      }
    break;

    /* Check Place piece / Castling / Capture*/
    case move_phase_1:     //----------------------------------------------------------------------------------------//
      /* Part is placed in new position*/
      if ((change_val > 0x79) && (piece_type == 0))
      {
        /*If a plyer lifted a piece and placed it on the same place(the player lifted by mistake),
        send error to APP and wait for resume - turn not changing*/
        if (current_state_board[place] != recovery_current_state_board[place])
        {
          return 0;
        }
        /* Update current_state_board with the part type. type_and_place was set in start_move state.*/
        current_state_board[place] = type_and_place[0][1];

        /* If suspect Castling */
        if (flag_castling)
        {
          DEBOUNCE_VAL = DEBOUNCE_VAL_slow;
          if (white_turn)
          {
            /*If white king moved to one of final and possible castling positions */
            if ((place == 0x02) || (place == 0x06))
            {
              /*Castling move for sure*/
              move_case = castling_phase_2;
              global_index = move_case;
            }
            else
            {
               /* If white king moved to one position on the way to castling.
                  At this point sure if it is castling or regulat move. */
              if ((place == 0x03) || (place == 0x05))
              {
                /* "castling-suspicious" phase */
                move_case = castling_phase_1;
                global_index = move_case;
                Flag_EOM_timer = true;
              }
              /* If the king didn't move to any of the "castling-suspicious' places, switch player */
              else
              {
                white_turn = false;
                /*Save state to recovery state*/
                flag_castling = false;
                move_case = start_move;
                //send_board(); //SEND BLE ARREY cmd 0X04 end of move
              }
            }
          }
          /* The same process with black player (just different cells on the board) */
          else
          {
            if ((place == 0x3A) || (place == 0x3E))
            {
              /*Castling move for sure*/
              move_case = castling_phase_2;
              global_index =move_case;
            }
            else if ((place == 0x3B) || (place == 0x3D))
            {
              /* "castling-suspicious" phase */
              move_case = castling_phase_1;
              Flag_EOM_timer = true;
              global_index =move_case;
            }
            /*End of move*/
            else
            {
              white_turn = true;
              flag_castling = false;
              move_case = start_move;
              //send_board(); //SEND BLE ARREY cmd 0X03 end of move
            }
          }
        }
        /*No castling(King didn't move), place piece*/
        else
        {
          // ------------- Transition of PAWN to QUEEN Shoud be checked (Not implemented)--------//

          // ------------------------------------------------------------------------------------//

          /* The piece was added */
          move_case = move_phase_2;///
          DEBOUNCE_VAL = DEBOUNCE_VAL_fast;
          flag_castling = false;
          Flag_EOM_timer = true; // if timer is up SEND BLE ARREY cmd 0X03 end of move
        }
      }
      /* Another part was lifted - Capture or castling */
      else if ((change_val < 0x79) && (piece_type!=0))
      {
        DEBOUNCE_VAL = DEBOUNCE_VAL_slow;
        if (white_turn)
        {
          /* White turn & black is lifted -> capture phase */
          if ((piece_type < 0x10))
          {
            move_case = capture_phase;
            flag_castling = false;
          }
          /* White turn & white ROOK is lifted -> castling phase */
          else if ((flag_castling) && (piece_type == ROOK_W))
          {
            /* Castling move for sure */
            move_case = castling_phase_2;
            global_index =move_case;
          }

          /* */
          else
          {
            /* Set the removed piece type to the position in current_state_board
             * type_and_place[0][0]: place of removed piece
             * type_and_place[0][1]: type of the removed piece
             */

             //current_state_board[type_and_place[0][0]] = type_and_place[0][1];
            return 0;
          }
          current_state_board[place] = 0; // clear the place of the lifted piece
        }
        /* Black turn - same process as the white */
        else
        {
          /* Black turn & white is lifted*/
          if (piece_type > 0x10)
          {
            move_case = capture_phase;
            flag_castling = false;
          }
          /* Black turn & Black ROOK is lifted -> castling phase */
          else if (flag_castling && (piece_type == ROOK_B))
          {
            /*Castling move for sure*/
            move_case = castling_phase_2;
            global_index =move_case;
          }
          else
          {
            current_state_board[type_and_place[0][0]] = type_and_place[0][1];  //
            return 0;
          }
          current_state_board[place] = 0;  // clear the place of the lifted piece
        }
      }
      /*type_and_place[2][x] - for debugging, not really used*/
      if (flag_castling)
      {
        type_and_place[2][0] = place;
        type_and_place[2][1] = piece_type;
      }
      else//put a piece on the board and not castling
      {
        type_and_place[1][0] = place;
        type_and_place[1][1] = piece_type;
      }
      break;

    case move_phase_2: //----------------------------------------------------------------------------------------------//
      EOM_counter = 0;
      DEBOUNCE_VAL = DEBOUNCE_VAL_slow;
      /* Piece was lifted */
      if ((change_val < 0x79)&&(piece_type!=0))
      {
        if (white_turn)
        {
          /*Black is lifted and white turn -> capture phase*/
          if ((piece_type < 0x10)) //
          {
            move_case = capture_phase;
          }
          /*The same part continue the move*/
          else if (place == type_and_place[1][0])
          {
              move_case = move_phase_1;
              Flag_EOM_timer = false;
              type_and_place[0][0] = place;
              type_and_place[0][1] = piece_type;
          }
          else// if another part is moved before end of move -> error
            return 0;
        }
        else
        {
           /* White is lifted and Black turn -> capture phase */
          if (piece_type > 0x10)
          {
            move_case = capture_phase;
          }
          /* The same part continue the move */
          else if (place == type_and_place[1][0])
          {
              move_case = move_phase_1;
              Flag_EOM_timer = false;
              type_and_place[0][0] == place;
              type_and_place[0][1] == piece_type;
          }
          else// if another part is moved before end of move -> error
            return 0;
        }
        current_state_board[place] = 0;
      }
      else
        return 0; //if Another piece was added - this is error
    break;

    case capture_phase://----------------------------------------------------------------------------------------------//
        /* If putting a piece in the same place as the removed opponent piece,
         * finish the move,update the board & go to start_move.
         * There is no rule check it the move was legal.
         */
        if ((change_val > 0x79)&&(piece_type==0)
            &&(place == type_and_place[1][0]))
        {
          // -------------PAWN to QUEEN(Not implemented)------//
          // -------------------------------------------------//

          current_state_board[place] = type_and_place[0][1]; // update current_state_board with the part type
          move_case = start_move;
          global_index =move_case;
          //send_board();              //SEND BLE ARREY cmd 0X03 end of move
          white_turn = !white_turn;
          type_and_place[1][0] = place;
          type_and_place[1][1] = type_and_place[0][1];

          send_EOM_flag = true;
          //send_end_of_move(white_turn);
        }
     break;

     case castling_phase_1://----------------------------------------------------------------------------------------------//
      /*
       * First time king moved in move_phase_1 and If king or casle moved another time it castling for sure
       * -> go to castling_phase_2
       */
      if ((change_val < 0x79)&&(piece_type!=0)) // king or castel are lifted
      {
        if (white_turn)
        {
          if ((piece_type == ROOK_W)||(piece_type == KING_W))
          {
            move_case = castling_phase_2;
            Flag_EOM_timer = false;
            current_state_board[place] = 0;
          }
          else
            return 0; // moved opponent's piece -> error
        }
        else if ((piece_type == ROOK_B)||(piece_type == KING_B))
        {
          move_case = castling_phase_2;
          Flag_EOM_timer = false;
          current_state_board[place] = 0;
        }
        else
          return 0;// moved opponent's piece -> error
      }
    break;

    case castling_phase_2:
      /*If piece was added*/
      if ((change_val > 0x7F)&&(piece_type==0))
      {
        if (white_turn)
        {
          if ((place == 0x02) || (place == 0x06))
          {
            current_state_board[place] = KING_W; // update current_state_board with the part type
          }
          else if ((place == 0x03) || (place == 0x05))
          {
            current_state_board[place] = ROOK_W; // update current_state_board with the part type
          }
          /*Check if the position after castling of White is OK-> finish the move.*/
          if ((((new_state[0] & 0xF0) == 0x60) && (current_state_board[0x06] == KING_W))||
              (((new_state[0] & 0x0F) == 0x0C) && (current_state_board[0x02] == KING_W))  )
          {
            move_case = start_move;
            global_index =move_case;
            flag_castling = false;
            //send_board();                 //SEND BLE ARREY cmd 0X03 end of move
            Flag_EOM_timer = false;       // clear flag
            EOM_counter = 0;
            white_turn = false;
            type_and_place[1][0] = place;
            type_and_place[1][1] = piece_type;
            send_EOM_flag = true;
            //send_end_of_move(white_turn);
          }
        }
        else
        {
          if ((place == 0x3A) || (place == 0x3E))
          {
            current_state_board[place] = KING_B; // update current_state_board with the part type
          }
          else if ((place == 0x3B) || (place == 0x3D))
          {
            current_state_board[place] = ROOK_B; // update current_state_board with the part type
          }
          /*Check if the position after castling of Black is OK-> finish the move.*/
          if ((((new_state[7] & 0xF0) == 0x60) && (current_state_board[0x3E] == KING_B ))||
              (((new_state[7] & 0x0F) == 0x0C) && (current_state_board[0x3A] == KING_B))   )
          {
            move_case = start_move;
            global_index =move_case;
            flag_castling = false;
            //send_board();                 //SEND BLE ARREY cmd 0X03 end of move
            Flag_EOM_timer = false;       // clear flag
            white_turn = true;
            type_and_place[1][0] = place;
            type_and_place[1][1] = piece_type;
            send_EOM_flag = true;
            //send_end_of_move(white_turn);
          }
        }
      }
      /*If king or rook was lifted -> clear the place */
      else if ((change_val < 0x7F)&&((piece_type==ROOK_B)||(piece_type==ROOK_W)||(piece_type==KING_B)||(piece_type==KING_W)))
      {
         current_state_board[place] =0;
      }
      else
        return 0;

      break;

      case end_move_phase_1:
      // This state is for debouncing of the placement of a piece
      /*If piece was added */
        if(change_val > 0x7F)
        {
          move_case = start_move;
          /*check if the piece places on the original location*/
          if (!(type_and_place[1][0] == place))
            return 0;
        }
        else
          return 0;
      break;

      default:
        move_case = start_move;
      break;
  }
//  send_board();
  return 1;
}

 //----------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------- Main ----------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
/**
 * @brief Application Main function
 * This function has the main loop of the application
 * Commands from the Android app are parsed and the appropriate function are called
 * @note pause_game_flag whick indicates if error occured is been checked
 *
 * @return int
 */
int main(void)
{
    bool erase_bonds;
    uint32_t ret;
    uint8_t data_from_adc[2];
    uint8_t line_val;
    int size_of_packet;
    uint8_t ind;
    uint8_t a, b;
    uint32_t gpio_P07_value;
    a = 1;

    init_dcb();
    uart_init();
    log_init();
    timers_init();
    #ifdef Debug_off_BLE_on
      //BLE Initialize.
      //buttons_leds_init(&erase_bonds);
      power_management_init();
      ble_stack_init();
      gap_params_init();
      gatt_init();
      services_init();
      advertising_init();
      conn_params_init();
      advertising_start();
    #endif
    saadc_init();
    uint8_t cs_ret;

  for(int i = 0; i<data_packet[2]; i++)
          data_packet[i] = 0xFF;
// Enter main loop.
    for (;;)
    {
        //nrf_pwr_mgmt_run();

        /*Check if data from Android recieved and the header(data_packet[0]) is correct*/
        if ((recieved_data_flag == true) && (data_packet[0] == header))
        {
          cs_ret = data_packet_cs();
          if (!data_packet_cs())
          {
              //Set flags to call appropriate functions according to the data from Android data_packet[1]
              switch (data_packet[1])
              {
                case ping_from_android:
                  send_ack(data_packet[1], true);
                break;

                case init_state:
//                  for (int i=0; i < 64 ; i++)
//                  {
//                    board_state_array[i] = data_packet[i+3];
//                  }
                    send_ack(data_packet[1], true);
                break;

                case start_of_game:
                  start_of_game_flag = true;
                  SOG_update_flag = true;
                  pause_game_flag = false;
                  send_ack(data_packet[1], true);
                break;

                case end_of_game:
                  start_of_game_flag = false;
                  send_ack(data_packet[1], true);
                break;

                case resume_game:
//                  send_ack(data_packet[1], true);
//                  pause_game_flag = false;


                  send_ack(data_packet[1], true);
                  /* Read the board before checking if the player changed
                   * the position of the pieces back*/

                  /*Pay attention!! some issue with read_board_position..
                    with one read function it's not working*/
                  read_board_position();
                  read_board_position();
                  read_board_position();
                  /*Compare the new board state with the recovered*/
                  if (compare_arrays(new_state, recovery_current_state, 8)==1)
                  {
                    /* If the player retured the pieces*/
                      pause_game_flag = false;
                      Flag_EOM_timer = false;
                      for (int i = 0; i < 64; i++ )
                      {
                        current_state_board[i] = recovery_current_state_board[i];
                      }
                  }
                  else
                  {
                    /* If the positions are not equal*/
                    pause_game_flag = true;
                    send_err(0,position_eror);
                  }
                break;
              }
              recieved_data_flag = false;
          }
            size_of_packet= data_packet[2] + 4;
            for(int i = 0; i<size_of_packet  ; i++)
            {
                 data_packet[i] = 0x00;
            }

        }

   #ifndef Debug_off_BLE_on
    start_of_game_flag = true;  //----------------------------------for debugging------------------------------------------ //
   #endif

      if (start_of_game_flag == true)
      {
        /*This is to enter here only once*/
        if (SOG_update_flag)
        {
          SOG_update_flag = false;

          for (uint8_t i=0; i<8; i++)
            for (uint8_t j=0; j<2; j++)
              change_array[i][j] = INIT_CHANGE_VALUE;
          line_val =0xFF;

          /* Update the current_state array */
          for ( ind=0; ind < 64 ; ind++)
          {
            /* Set the current_state_board with inital board state*/
            current_state_board[ind]=board_state_array[ind];
            recovery_current_state_board[ind] = board_state_array[ind];

            /* Init the current_state[8] array, holds the
             * occupied cells in the board. Every bit is one cell*/
            if (board_state_array[ind]==0x00)
              line_val = line_val & NBIT(ind % BITS_IN_BYTE);  // clear bit
            if(((ind+1) % BITS_IN_BYTE) == 0)
            {
              current_state[((ind+1) / BITS_IN_BYTE)-1] = line_val;
              recovery_current_state[((ind+1) / BITS_IN_BYTE)-1] = line_val;
              line_val =0xFF;
            }
          }

          /*White is first to move*/
          white_turn = true;
          move_case = start_move;
        }
        /* If there is no error, continue to operate*/
        if(!pause_game_flag)
        {
          /* Read gpio to set Threshhold for ADC*/
          gpio_P07_value = nrf_gpio_pin_read(SW_BLACK);
          if (gpio_P07_value == 1)
            adc_treshold = HIGH_ADC_THRESHOLD;
          else
            adc_treshold = LOW_ADC_THRESHOLD;


          /*Read board and set the new_state[8] array.*/
          read_board_position();

          // for(uint8_t i = 0; i<8; i++)
          //   app_uart_put(0xa0+i);
          //for(uint8_t i = 0; i<8; i++)
          //  app_uart_put(current_state[i]);
          //app_uart_put(0xff);
          nrf_delay_us(500);

          /*This function will compare new_state[8] with current_state[8]*/
          compare_block();

          if (Flag_EOM_timer)
          {
            nrf_gpio_pin_set(TP1);
            ++EOM_counter;

            /* EOM_counter act as timer to finish the move,
             * when reaches EOM_THRESHOLD-> swich move
             */
            if (EOM_counter > EOM_THRESHOLD)
            {
              EOM_counter =0;
              Flag_EOM_timer = false;
              flag_castling = false;
              move_case = start_move;
              DEBOUNCE_VAL = DEBOUNCE_VAL_slow;
              white_turn =! white_turn; /*Change player*/
              send_end_of_move(white_turn); /*send to android*/
              nrf_gpio_pin_clear(TP1);
            }
          }
          else
            EOM_counter =0;
        }
      }
      //idle_state_handle();
    }
}


/**
 * @}
 */