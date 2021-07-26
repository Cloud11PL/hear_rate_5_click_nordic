#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_spo.h"
#include "nrf_log.h"

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2
#define MAX_SPO_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)

#define INITIAL_VALUE_SPO 0x00

// Heart Rate Measurement flag bits
#define SPO_FLAG_MASK_HR_VALUE_16BIT            (0x01 << 0)                           /**< Heart Rate Value Format bit. */
#define SPO_FLAG_MASK_SENSOR_CONTACT_DETECTED   (0x01 << 1)                           /**< Sensor Contact Detected bit. */
#define SPO_FLAG_MASK_SENSOR_CONTACT_SUPPORTED  (0x01 << 2)                           /**< Sensor Contact Supported bit. */
#define SPO_FLAG_MASK_EXPENDED_ENERGY_INCLUDED  (0x01 << 3)                           /**< Energy Expended Status bit. Feature Not Supported */
#define SPO_FLAG_MASK_RR_INTERVAL_INCLUDED      (0x01 << 4)                           /**< RR-Interval bit. */

static void on_connect(ble_spo_t * p_spo, ble_evt_t const * p_ble_evt)
{
    p_spo->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


static void on_disconnect(ble_spo_t * p_spo, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_spo->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_spo_cccd_write(ble_spo_t * p_spo, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_spo->evt_handler != NULL)
        {
            ble_spo_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_spo_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_spo_EVT_NOTIFICATION_DISABLED;
            }

            p_spo->evt_handler(p_spo, &evt);
        }
    }
}

static void on_write(ble_spo_t * p_spo, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_spo->spo_handles.cccd_handle)
    {
        on_spo_cccd_write(p_spo, p_evt_write);
    }
}


void ble_spo_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_spo_t * p_spo = (ble_spo_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_spo, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_spo, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_spo, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


static uint8_t spo_encode(ble_spo_t * p_spo, uint32_t data, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Set sensor contact related flags
    //if (p_spo->is_sensor_contact_supported)
    //{
    //    flags |= SPO_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    //}
    //if (p_spo->is_sensor_contact_detected)
    //{
    //    flags |= SPO_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    //}

    // Encode heart rate measurement
    if (data > 0xff)
    {
        flags |= SPO_FLAG_MASK_HR_VALUE_16BIT;
        len   += uint16_encode(data, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)data;
    }

    // Encode rr_interval values
    //if (p_spo->rr_interval_count > 0)
    //{
    //    flags |= SPO_FLAG_MASK_RR_INTERVAL_INCLUDED;
    //}
    //for (i = 0; i < p_spo->rr_interval_count; i++)
    //{
    //    if (len + sizeof(uint16_t) > p_spo->max_spo_len)
    //    {
    //        // Not all stored rr_interval values can fit into the encoded hrm,
    //        // move the remaining values to the start of the buffer.
    //        memmove(&p_spo->rr_interval[0],
    //                &p_spo->rr_interval[i],
    //                (p_spo->rr_interval_count - i) * sizeof(uint16_t));
    //        break;
    //    }
    //    len += uint16_encode(p_spo->rr_interval[i], &p_encoded_buffer[len]);
    //}
    //p_spo->rr_interval_count -= i;

    // Add flags
    p_encoded_buffer[0] = flags;
    //p_encoded_buffer[1] = data >> 8;
    //p_encoded_buffer[2] = data >> 16;
    //p_encoded_buffer[3] = data >> 24;

    return len;
}


uint32_t ble_spo_init(ble_spo_t * p_spo, const ble_spo_init_t * p_spo_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_hrm[MAX_SPO_LEN];

    // Initialize service structure
    p_spo->evt_handler                 = p_spo_init->evt_handler;
    p_spo->is_sensor_contact_supported = p_spo_init->is_sensor_contact_supported;
    p_spo->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    //p_spo->is_sensor_contact_detected  = false;
    //p_spo->rr_interval_count           = 0;
    p_spo->max_spo_len                 = MAX_SPO_LEN;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_PLX_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_spo->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add heart rate measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_PLX_CONTINUOUS_MEAS;
    //add_char_params.uuid              = BLE_UUID_HEART_RATE_MEASUREMENT_CHAR;
    add_char_params.max_len           = MAX_SPO_LEN;
    add_char_params.init_len          = spo_encode(p_spo, INITIAL_VALUE_SPO, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_spo_init->spo_cccd_wr_sec;

    err_code = characteristic_add(p_spo->service_handle, &add_char_params, &(p_spo->spo_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    //if (p_spo_init->p_body_sensor_location != NULL)
    //{
    //    // Add body sensor location characteristic
    //    memset(&add_char_params, 0, sizeof(add_char_params));

    //    add_char_params.uuid            = BLE_UUID_BODY_SENSOR_LOCATION_CHAR;
    //    add_char_params.max_len         = sizeof(uint8_t);
    //    add_char_params.init_len        = sizeof(uint8_t);
    //    add_char_params.p_init_value    = p_spo_init->p_body_sensor_location;
    //    add_char_params.char_props.read = 1;
    //    add_char_params.read_access     = p_spo_init->bsl_rd_sec;

    //    err_code = characteristic_add(p_spo->service_handle, &add_char_params, &(p_spo->bsl_handles));
    //    if (err_code != NRF_SUCCESS)
    //    {
    //        return err_code;
    //    }
    //}

    return NRF_SUCCESS;
}


uint32_t ble_spo_data_measurement_send(ble_spo_t * p_spo, uint16_t data)
{
    NRF_LOG_INFO("%d", data);
    //NRF_LOG_FLUSH();

    uint32_t err_code;

    // Send value if connected and notifying
    if (p_spo->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hrm[MAX_SPO_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = spo_encode(p_spo, data, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_spo->spo_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_spo->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


//void ble_spo_rr_interval_add(ble_spo_t * p_spo, uint16_t rr_interval)
//{
//    if (p_spo->rr_interval_count == BLE_spo_MAX_BUFFERED_RR_INTERVALS)
//    {
//        // The rr_interval buffer is full, delete the oldest value
//        memmove(&p_spo->rr_interval[0],
//                &p_spo->rr_interval[1],
//                (BLE_spo_MAX_BUFFERED_RR_INTERVALS - 1) * sizeof(uint16_t));
//        p_spo->rr_interval_count--;
//    }

//    // Add new value
//    p_spo->rr_interval[p_spo->rr_interval_count++] = rr_interval;
//}


//bool ble_spo_rr_interval_buffer_is_full(ble_spo_t * p_spo)
//{
//    return (p_spo->rr_interval_count == BLE_spo_MAX_BUFFERED_RR_INTERVALS);
//}


uint32_t ble_spo_sensor_contact_supported_set(ble_spo_t * p_spo, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_spo->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_spo->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}


//void ble_spo_sensor_contact_detected_update(ble_spo_t * p_spo, bool is_sensor_contact_detected)
//{
//    p_spo->is_sensor_contact_detected = is_sensor_contact_detected;
//}


uint32_t ble_spo_body_sensor_location_set(ble_spo_t * p_spo, uint8_t body_sensor_location)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &body_sensor_location;

    return sd_ble_gatts_value_set(p_spo->conn_handle, p_spo->bsl_handles.value_handle, &gatts_value);
}


void ble_spo_on_gatt_evt(ble_spo_t * p_spo, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_spo->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_spo->max_spo_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}