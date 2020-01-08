
// -----------------DCB Digital Chess Board ,all code functions are in this file ------------------ 
// 2019_09_08_V00  first version for demo                           //
// 2019_09_08_V01  fix ADC to n one block mode                      //
//                 fixed bugs in the new_position decleration       //  


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
#define SAADC_CALIBRATION_INTERVAL 5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
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
 
#define PAWNS_W         0x11
#define ROOK_W 		0x12
#define KNIGHT_W 	0x13 
#define BISHOP_W 	0x14 
#define QUEEN_W         0x15 
#define KING_W 		0x16 


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

#define INIT_CHANGE_VALUE 0xFF
#define MAX_NUMBER_OF_CHANGES 8
#define BLOCK_NUM 8
#define NUM_OF_CELLS 64
#define BITS_IN_BYTE 8
#define BIT(n) (0x1U <<(n))

#define PHOTO_DIODE_DELAY_US 5
#define ANALOG_MUX_DELAY_US 5
#define DEBOUNCE_VAL 5
#define THRESHOLD 70



enum command_value{
ping_from_dcb = 0x01,
mask_bit_array = 0x02,
full_bit_array = 0x03,
end_of_move = 0x04,
//---------------------
ping_from_android =0x80,
init_state = 0x81,
start_of_game = 0x82,
end_of_game = 0x83,
header =  0xAA
};

enum move_algo_case{
start_move        =1,
move_phase_1      =2,
move_phase_2      =3,
capture_phase     =4,
castling_phase_1  =5,
castling_phase_2  =6
};

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                /**< Advertising module instance. */
void saadc_sampling_event_init(void);


/*--Function decalraitions--*/
int set_block(uint8_t block_num);
int set_addr(uint8_t addr_num);
int set_analog_mux(uint8_t ana_num);
int read_adc(_Bool *data);
int init_dcb();
int move_algo(uint8_t change_val, _Bool reset);
int enable_pd_block(uint8_t block_num);
int read_block();
int read_board_position();
void board_test();
int compare_block();//(uint8_t *current_state, uint8_t *new_state);

int send_board(uint8_t index);


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

volatile uint8_t* data_packet; 
uint8_t* data_to_send;

/*
m_buffer_pool[0][1], m_buffer_pool[1][1] : channel AN0
m_buffer_pool[0][0], m_buffer_pool[1][0] : channel AN1 
*/
static nrf_saadc_value_t       m_buffer_pool[1][SAADC_SAMPLES_IN_BUFFER];

static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_calibrate = false;      


/*-- Global Variables--*/
uint8_t change_array[4];
uint8_t current_position[8]; 
uint8_t current_state[8];
uint8_t new_position[8]; // Holds the new board state 
uint8_t new_state[8];
uint8_t current_state_board[64] = {0};

_Bool position_test[64]; 

void send_end_of_move(_Bool white);


//volatile bool sampling_flag = false;


volatile uint16_t sampling_data;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

volatile _Bool recieved_data_flag = false;
volatile _Bool white_turn = true;
volatile bool ADC_busy_Flag;
_Bool start_of_game_flag = false;


uint8_t board_state_array[64] = {
  ROOK_W, KNIGHT_W, BISHOP_W, QUEEN_W, KING_W, BISHOP_W, KNIGHT_W, ROOK_W,
  PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W,
  0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,
  PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, 
  ROOK_B,KNIGHT_B,BISHOP_B,QUEEN_B,KING_B, BISHOP_B, KNIGHT_B, ROOK_B};





uint8_t board_state_array1[64] = {
  ROOK_W, KNIGHT_W, BISHOP_W, QUEEN_W, KING_W, BISHOP_W, KNIGHT_W, ROOK_W,
  PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, 0, PAWNS_W, PAWNS_W, PAWNS_W,
  0,0,0,0,0,0,0,0,
  0,0,0,0,PAWNS_W,0,0,0,
  0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,
  PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B, 
  ROOK_B,KNIGHT_B,BISHOP_B,QUEEN_B,KING_B, BISHOP_B, KNIGHT_B, ROOK_B};


uint8_t board_state_array2[64] = {
  ROOK_W, KNIGHT_W, BISHOP_W, QUEEN_W, KING_W, BISHOP_W, KNIGHT_W, ROOK_W,
  PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, 0, PAWNS_W, PAWNS_W, PAWNS_W,
  0,0,0,0,0,0,0,0,
  0,0,0,0,PAWNS_W,0,0,0,
  0,0,0,0,PAWNS_B,0,0,0,
  0,0,0,0,0,0,0,0,
  PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B,  0, PAWNS_B, PAWNS_B, PAWNS_B, 
  ROOK_B,KNIGHT_B,BISHOP_B,QUEEN_B,KING_B, BISHOP_B, KNIGHT_B, ROOK_B};



uint8_t board_state_array3[64] = {
  ROOK_W, 0, BISHOP_W, QUEEN_W, KING_W, BISHOP_W, KNIGHT_W, ROOK_W,
  PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, 0, PAWNS_W, PAWNS_W, PAWNS_W,
  0,0,KNIGHT_W,0,0,0,0,0,
  0,0,0,0,PAWNS_W,0,0,0,
  0,0,0,0,PAWNS_B,0,0,0,
  0,0,0,0,0,0,0,0,
  PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B,  0, PAWNS_B, PAWNS_B, PAWNS_B, 
  ROOK_B,KNIGHT_B,BISHOP_B,QUEEN_B,KING_B, BISHOP_B, KNIGHT_B, ROOK_B};



uint8_t board_state_array4[64] = {
  ROOK_W, 0, BISHOP_W, QUEEN_W, KING_W, BISHOP_W, KNIGHT_W, ROOK_W,
  PAWNS_W, PAWNS_W, PAWNS_W, PAWNS_W, 0, PAWNS_W, PAWNS_W, PAWNS_W,
  0,0,KNIGHT_W,0,0,0,0,0,
  0,0,0,0,PAWNS_W,0,0,0,
  0,0,0,0,PAWNS_B,0,0,0,
  0,0,0,0,0,KNIGHT_B,0,0,
  PAWNS_B, PAWNS_B, PAWNS_B, PAWNS_B,  0, PAWNS_B, PAWNS_B, PAWNS_B, 
  ROOK_B,KNIGHT_B,BISHOP_B,QUEEN_B,KING_B, BISHOP_B, 0, ROOK_B};


uint8_t board_lookup[32] = { 0,    //[0,0,0,0,0,0,0,0],
                            4,    //[0,0,0,0,0,1,0,0],
                            8,    //[0,0,0,0,1,0,0,0],
                            12,   //[0,0,0,0,1,1,0,0],
                            66,   //[0,1,0,0,0,0,1,0],
                            70,   //[0,1,0,0,0,1,1,0],
                            74,   //[0,1,0,0,1,0,1,0],
                            78,   //[0,1,0,0,1,1,1,0],
                            16,   //[0,0,0,1,0,0,0,0],
                            20,   //[0,0,0,1,0,1,0,0],
                            24,   //[0,0,0,1,1,0,0,0],
                            28,   //[0,0,0,1,1,1,0,0],
                            82,   //[0,1,0,1,0,0,1,0],
                            86,   //[0,1,0,1,0,1,1,0],
                            90,   //[0,1,0,1,1,0,1,0],
                            94,   //[0,1,0,1,1,1,1,0],
                            33,   //[0,0,1,0,0,0,0,1],
                            37,   //[0,0,1,0,0,1,0,1],
                            41,   //[0,0,1,0,1,0,0,1],
                            45,   //[0,0,1,0,1,1,0,1],
                            99,   //[0,1,1,0,0,0,1,1],
                            103,  //[0,1,1,0,0,1,1,1],
                            107,  //[0,1,1,0,1,0,1,1],
                            111,  //[0,1,1,0,1,1,1,1],
                            46,   //[0,0,1,1,0,0,0,1],
                            53,   //[0,0,1,1,0,1,0,1],
                            57,   //[0,0,1,1,1,0,0,1],
                            61,   //[0,0,1,1,1,1,0,1],
                            115,  //[0,1,1,1,0,0,1,1],
                            119,  //[0,1,1,1,0,1,1,1],
                            123,  //[0,1,1,1,1,0,1,1],
                            127,  //[0,1,1,1,1,1,1,1]
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

    data_packet = (uint8_t*)malloc(sizeof(uint8_t) * p_evt->params.rx_data.length);

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


/**@brief Function for handling events from the GATT library. */
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
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
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
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
      sampling_data = p_event->data.done.p_buffer[0];   
      //sampling_flag = true;       
      err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again.         
      APP_ERROR_CHECK(err_code);
      ADC_busy_Flag = 0;  
//      nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
//      nrf_saadc_event_clear(NRF_SAADC_EVENT_END);    
      m_adc_evt_counter++;
    }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------saadc_init-------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
void saadc_init(void)
{
     ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config0;
    nrf_saadc_channel_config_t channel_config2;
	
    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = 0;                                                          //no over sample.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config0.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config0.gain = NRF_SAADC_GAIN1_4;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config0.acq_time = NRF_SAADC_ACQTIME_5US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config0.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config0.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config0.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config0.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config0.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin


    //Configure SAADC channel
    channel_config2.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config2.gain = NRF_SAADC_GAIN1_4;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config2.acq_time = NRF_SAADC_ACQTIME_5US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config2.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config2.pin_p = NRF_SAADC_INPUT_AIN2;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config2.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config2.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config2.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_config2);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);

}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------data_packet_cs---------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
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
//
//int set_board_init(Data_packet* packet)
//{
//  packet->header = 0xAA;
//  packet->command = 0x03;      
//  packet->length = sizeof(packet->data);
//  memcpy(packet->data, board_state_array, packet->length);
//  data_packet_cs(packet);
//  return 0;
//
//
//}

//---------------------------------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------send_ack---------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
uint32_t send_ack(uint8_t command, _Bool ACK)
{  
  data_to_send = (uint8_t*) malloc(sizeof(uint8_t) * 6);
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
void send_end_of_move(_Bool white)
{
  uint8_t data_to_send1[5] = {0};
  uint16_t length;
  uint8_t checksum = 0;
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
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------send_board------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
int send_board(uint8_t index)
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
        switch(index)
        {
         case 1:
            data_to_send1[i + 3] = board_state_array1[i];
         break;
         case 2:
            data_to_send1[i + 3] = board_state_array2[i];
         break;
         case 3:
            data_to_send1[i + 3] = board_state_array3[i];
         break;
         case 4:
            data_to_send1[i + 3] = board_state_array4[i];
         break;
        }      
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
//------------------------------------------------------------------------init_dcb-------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//

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

  nrf_gpio_cfg_output(EN_PD);  
  nrf_gpio_cfg_output(EN_AMP);
  nrf_gpio_cfg_output(EN_BLOCK);
  nrf_gpio_cfg_output(EN_SYS);
  nrf_gpio_cfg_output(TP1);
  nrf_gpio_cfg_output(TP2);
  read_board_position();

  for(i = 0; i < NUM_OF_CELLS / 8; i++)
    current_position[i] = 0;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------board_test---------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------------------------------------------//
void board_test()
{    
    nrf_gpio_pin_set(EN_SYS);
    nrf_gpio_pin_set(EN_AMP);        
    nrf_gpio_pin_set(EN_BLOCK);
    nrf_gpio_pin_set(EN_PD);
     
    set_block(board_lookup[0] >> 4);    
    set_addr((board_lookup[0] >> 2) & 0x03);   
    set_analog_mux(board_lookup[0] & 0x03); 
  
    nrf_delay_us(300);
  //read
    
    set_block(board_lookup[4] >> 4);    
    set_addr((board_lookup[4] >> 2) & 0x03);   
    set_analog_mux(board_lookup[4] & 0x03);
    
    nrf_delay_us(200);

    nrf_gpio_pin_clear(EN_BLOCK);
    nrf_delay_ms(2);  
}

//----------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------set_addr------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//

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

int read_board_position()
{
  int index;
  int a,b;

  uint8_t line_val0;
  uint8_t line_val1;
 
  _Bool adc_val[2];
  a = 1;
 

  nrf_gpio_pin_set(EN_SYS);
  nrf_gpio_pin_set(EN_PD);
  nrf_gpio_pin_set(EN_AMP);
  nrf_gpio_pin_set(EN_BLOCK);
  nrf_gpio_pin_set(TP1);
  nrf_delay_us(500);
  
  /* Scanning the board 32 times and each 
  * time get 2 cells at a time
  */
  for(uint8_t i = 0; i<8; i++)
    new_position[i] = 0;
  for (index = 0; index < NUM_OF_CELLS/2; index++)
  { 
    nrf_gpio_pin_set(TP2);

    set_block(board_lookup[index] >> 4);
    //nrf_delay_us (PHOTO_DIODE_DELAY_US);

    set_addr((board_lookup[index] >> 2) & 3);   
    set_analog_mux((board_lookup[index]) & 3);

    //nrf_delay_us(ANALOG_MUX_DELAY_US);
    b=a;
    nrf_gpio_pin_clear(TP2);
    nrf_delay_us(20);
    read_adc(adc_val);    
    position_test[index * 2] = adc_val[0];
    position_test[(index * 2) + 1] = adc_val[1];

    if (adc_val[0] == true)
      line_val0 = line_val0 | BIT(index % BITS_IN_BYTE);  // set bit      

    if (adc_val[1] == true)
      line_val1 = line_val1 | BIT(index % BITS_IN_BYTE); // set bit   

    if(index % BITS_IN_BYTE == 0)  // if scanned a line - update the array
    {
      new_position[2 * index / BITS_IN_BYTE] = line_val0;
      new_position[(2 * index / BITS_IN_BYTE) + 1] = line_val1;
    } 
    
  }

b = a;
a = b;


  nrf_gpio_pin_clear(EN_PD);
  nrf_gpio_pin_clear(EN_BLOCK); 
  nrf_gpio_pin_clear(EN_AMP);
  nrf_gpio_pin_clear(EN_SYS);
  nrf_gpio_pin_clear(TP1);
  return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------ compare_block -------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
int compare_block()//(uint8_t *current_state, uint8_t *new_state)
{
  int i,j;
  uint8_t line;
  uint8_t column;
  uint8_t current_bit;
  uint8_t new_bit;
  uint8_t change_location;
  uint8_t change;
  bool flag_new_change;

 /*
  * change_array [[change_val],[0]]      
  * change_array [[change_counter],[1]] 
  */
  uint8_t change_array[8][2];
  
  for (i=0; i<8; i++)
    for (j=0; j<2; j++)
     change_array[i][j] = INIT_CHANGE_VALUE;


  for (line = 0; line < 8; line++)
  {
    if (current_state[line] != new_state[line])	
    {
      for(column = 0; column < 8; column++)
      {
        current_bit = (current_state[line] >> column) & 0x01 ;
        new_bit = (new_state[line] >> column) & 0x01 ;
        if(current_bit != new_bit);
        {
          //change_val structure [1(added)/0(removed), col2,col1,col0,0, line2, line1, line0] 
          change_location = line + column << 4; 
          
          if (new_bit) // a piece was placed on the squre
            change_location = change_location | 0x80; 

          flag_new_change = true;
          for(change = 0; change < MAX_NUMBER_OF_CHANGES; change++) 
          {
            if (change_array[change][0] == change_location)
            {	
                /* ADD 2 in case of change and subtruct 1 from everybody later so
                   eventually add 1 to the new changes.
                 */
                change_array [change][1] = change_array[change][1] + 2;                              
                flag_new_change = false;
                break; // ---- verify it isn't exiting the function but only the loop---
            }
          }

          if (flag_new_change) // if the change happned for the first time.
          {
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
      if (change_array[change][0]!=INIT_CHANGE_VALUE)
      {	
        if (change_array[change][1] == 0x00)
        {
            change_array[change][0]==INIT_CHANGE_VALUE;
            change_array[change][1]==INIT_CHANGE_VALUE;
        }
        else
        {
          --change_array[change][1];
        }
      }
      if (change_array[change][1] > DEBOUNCE_VAL)
      {
        move_algo(change_location,false);//uint8 change_array[change][0]);
        change_array[change][0]==INIT_CHANGE_VALUE;
        change_array[change][1]==INIT_CHANGE_VALUE;
      }
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------Read ADC -------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
int read_adc(_Bool *val)
{
    int temp;
    static uint8_t counter; 
    uint8_t x_val,y_val; 
  
    ADC_busy_Flag=1;
    nrf_drv_saadc_sample();
    counter = 0; 
    while (ADC_busy_Flag && (counter< 50))
    {
      ++counter;
      nrf_delay_us(1);
    }
    if( m_buffer_pool[0][1] < 0)
        m_buffer_pool[0][1] = 0;
    if( m_buffer_pool[0][0] < 0)
        m_buffer_pool[0][0] = 0;
    
    if (m_buffer_pool[0][1] < THRESHOLD) // AN2 chanel y 
        val[0] = true;
    else 
        val[0] = false;
      

    if (m_buffer_pool[0][0] < THRESHOLD)//AN0 chanel X
        val[1] = true;
    else 
        val[1] = false;  
  
    //sampling_flag = false;
    //--------------- Debug only ----------------------//
    x_val = m_buffer_pool[0][0]>>2;
    //app_uart_put(x_val); 
    //nrf_delay_ms(1); 
    y_val = m_buffer_pool[0][1]>>2;
    //app_uart_put(y_val); 

    //--------------- Debug only ----------------------//
   return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------- move_algo -------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
int move_algo(uint8_t change_val, _Bool reset) 
{
  static uint8_t white_turn;
  static uint8_t move_case;
  static uint8_t type_and_place[3][2];
  uint8_t place;
  uint8_t piece_type;
  _Bool flag_castling = false;

  if (reset == true)
  {
    white_turn = ~white_turn;
    move_case = start_move;
    for (int i=0; i<3; i++)
      for (int j=0; j<2; j++)
        type_and_place[i][j] = 0;    
  }

  place = (change_val & 0x30) >> 1 + change_val & 0x03; //0-64 posision and type
  piece_type = current_state_board[place];
  //reset_timer_EOM = True; // reset the counter for END of move
  
  switch (move_case) 
  {
  case start_move:
    type_and_place[0][0] = place;      //
    type_and_place[0][1] = piece_type; //
    
    if (change_val < 0x79)             // remove piece
    {
      current_state_board[place] = 0;
      if (white_turn)
      {
        if ((piece_type == KING_W) && (place == 0x04)) // check for Castling
          flag_castling = true;
        move_case = move_phase_1;
      } 
      else
      {
        if ((piece_type == KING_B) && (place == 0x3C)) // check for Castling
          flag_castling = true;
        move_case = move_phase_1;
      }
    }
  break;

  case move_phase_1:     // place piece / Castling/ capture
    if (change_val > 0x79) // part is placed in new position
    {
      
      current_state_board[place] = type_and_place[0][1]; // update current_state_board with the part type
      if (flag_castling)                      //Castling
      {
        if (white_turn) 
        {
          if ((place == 0x02) || (place == 0x06))
            move_case = castling_phase_2;
          else 
          {
            if ((place == 0x03) || (place == 0x05)) 
            {
              move_case = castling_phase_1;
              //Flag_EOM_timer = true;
            } 
            else 
            {
              white_turn = false;
              move_case = start_move;
              flag_castling = false;
              //SEND BLE ARREY cmd 0X03 end of move
            }
          }
        } 
        else 
        {
          if ((place == 0x38) || (place == 0x3E)) 
          {
            move_case = castling_phase_2;
          } 
          else if ((place == 0x39) || (place == 0x3D)) 
          {
            move_case = castling_phase_1;
            //Flag_EOM_timer = TRUE;
          } 
          else 
          {
            white_turn = true;
            move_case = start_move;
            flag_castling = false;
            //SEND BLE ARREY cmd 0X03 end of move
          }
        }
      } 
      else //place piece
      {
        move_case = move_phase_2;///??
        flag_castling = false;
        //Flag_EOM_timer = true; // if timer is up SEND BLE ARREY cmd 0X03 end of move
      }

    } 
    else // another part was lifted
    {
      current_state_board[place] = 0;
      if (white_turn) 
      {
        if ((piece_type < 0x10)) // black is lifted
        {
          move_case = capture_phase;
          flag_castling = false;
        } 
        else if ((flag_castling) && (piece_type == ROOK_W))
          move_case = castling_phase_1;                    
//        else
//          error; ///tbd
      }
      else 
      {
        if (piece_type > 0x10) // white is lifted
        {
          move_case = capture_phase;
          flag_castling = false;
        } 
        else if (flag_castling && (piece_type == ROOK_B))
          move_case = castling_phase_1;
//        else
//          error; ///
      }

      if (flag_castling) 
      {
        type_and_place[2][0] = place;
        type_and_place[2][1] = piece_type;
      } 
      else 
      {
        type_and_place[1][0] = place;
        type_and_place[1][1] = piece_type;
      }
    }
    break;

    case move_phase_2:      
      if (change_val < 0x79)             // remove piece
      {
        current_state_board[place] = 0;                
        move_case = move_phase_1;
            
      }
    break;

    case capture_phase:
        if ((change_val > 0x79)
          &&(place == type_and_place[1][0])) {
            current_state_board[place] = type_and_place[0][1]; // update current_state_board with the part type
            move_case = start_move;
            // clear type_and_place array
            //SEND BLE ARREY cmd 0X03 end of move
          }
     break;

     case castling_phase_1:
        current_state_board[place] = type_and_place[0][1]; // update CP_array with the part type

        if (change_val < 0x79) // king or castel  are lifted
        {
          current_state_board[place] = 0;
          if (white_turn) 
          {
            if (piece_type < 0x10)
              if ((place == type_and_place[2][0])
                || (place == 0x00) || (place == 0x07)) 
                move_case = castling_phase_2;

          } 
          else if (piece_type > 0x10) 
          {
            if ((place == type_and_place[2][0])
              || (place == 0x38) || (place == 0x3F))
              move_case = castling_phase_2;
          } 
//          else
//            Error;
        }

      break;

      case castling_phase_2:
        if (change_val > 0x7F) 
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

            if (((current_state[0] & 0x60) == 0x60) || ((current_state[0] & 0x0C) == 0x0C))
            {
              move_case = start_move;
              // clear type_and_place array
              //SEND BLE ARREY cmd 0X03 end of move
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
            if (((current_state[7] & 0x60) == 0x60) || ((current_state[7] & 0x0C) == 0x0C)) 
            {
              move_case = start_move;
              // clear type_and_place array
              //SEND BLE ARREY cmd 0X03 end of move
            }
          }
        }
        break;

      default:
        move_case = start_move;
        break;
      }
    }


/**@brief Application main function.
 */
 //----------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------- Main ----------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
int main(void)
{
    bool erase_bonds;
    uint32_t ret;
    uint8_t data_from_adc[2];
    int size_of_packet;
    uint8_t a, b;
    a = 1;
    a = 1;
    // BLE Initialize.
    uart_init();
    log_init();
    timers_init();
//    buttons_leds_init(&erase_bonds);
//    power_management_init();
//    ble_stack_init();
//    gap_params_init();
//    gatt_init();
//    services_init();
//    advertising_init();
//    conn_params_init();
      init_dcb();
//    advertising_start();       
    saadc_init();    
    uint8_t* pboard_state;
    uint8_t cs_ret;
    //set_board_init(&init_board_state);
   
    for(int i = 0; i<data_packet[2]; i++)
            data_packet[i] = 0xFF;
// Enter main loop.
    for (;;)
    {       

       if(m_saadc_calibrate == true)
        {
            while(nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS); //Trigger calibration task
            m_saadc_calibrate = false;
        }

        nrf_pwr_mgmt_run();

        if ((recieved_data_flag == true) && (data_packet[0] == header))
        {
          cs_ret = data_packet_cs();
          if (!data_packet_cs())
          {
              switch (data_packet[1])
              {
                case ping_from_android:                   
                  send_ack(data_packet[1], true);  
                break;
              
                case init_state:
                  for (int i=0; i < 64 ; i++)
                    board_state_array[i] = data_packet[i+3]; 
                    send_ack(data_packet[1], true);                    
                break;   
               
                case start_of_game:                
                  start_of_game_flag = true;                                   
                  send_ack(data_packet[1], true); 
                break;      
              
                case end_of_game:    
                  start_of_game_flag = false;
                  send_ack(data_packet[1], true);  
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


    start_of_game_flag = true;  //----------------------------------for debugging------------------------------------------ //

    if (start_of_game_flag == true)
    {
      read_board_position();
      //for(uint8_t i = 0; i<8; i++)
      //   app_uart_put(0xa0+i);  
       for(uint8_t i = 0; i<8; i++)
         app_uart_put(new_position[i]);  
      app_uart_put(0xaa);      
      nrf_delay_ms(500);       
     // compare_block(); 
//      nrf_delay_ms(1500);  
//      send_board(1);
//      send_end_of_move(false);
//      nrf_delay_ms(2221);     
//
//      send_board(2);
//      send_end_of_move(true);
//      nrf_delay_ms(1540); 
//
//      send_board(3);
//      send_end_of_move(false);
//      nrf_delay_ms(2020); 
//
//      send_board(4);
//      send_end_of_move(true);
//      nrf_delay_ms(4020);             
      }

   //   board_test();
      
      //idle_state_handle(); 
        
    }
}


/**
 * @}
 */