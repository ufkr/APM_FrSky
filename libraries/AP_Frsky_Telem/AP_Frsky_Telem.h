// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Motors/AP_Motors.h>

#define MSG_BUFFER_LENGTH       5 // size of the message buffer queue (number of messages waiting to be sent)

/* 
 for FrSky D protocol (D-receivers)
*/
// FrSky sensor hub data IDs (1 byte) */
#define GPS_ALT_BP_ID             0x01
#define TEMP1_ID                  0x02
#define FUEL_ID                   0x04
#define TEMP2_ID                  0x05
#define GPS_ALT_AP_ID             0x09
#define BARO_ALT_BP_ID            0x10
#define GPS_SPEED_BP_ID           0x11
#define GPS_LONG_BP_ID            0x12
#define GPS_LAT_BP_ID             0x13
#define GPS_COURS_BP_ID           0x14
#define GPS_SPEED_AP_ID           0x19
#define GPS_LONG_AP_ID            0x1A
#define GPS_LAT_AP_ID             0x1B
#define BARO_ALT_AP_ID            0x21
#define GPS_LONG_EW_ID            0x22
#define GPS_LAT_NS_ID             0x23
#define CURRENT_ID                0x28
#define VFAS_ID                   0x39

/* 
 for FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
*/
// FrSky PRIM IDs (1 byte)
#define DATA_FRAME             0x10

#define START_STOP_SPORT       0x7E

// FrSky DATA IDs (2 bytes)
#define ALT_FIRST_ID           0x0100
#define VARIO_FIRST_ID         0x0110
#define VFAS_FIRST_ID          0x0210
#define GPS_LONG_LATI_FIRST_ID 0x0800
#define DIY_FIRST_ID           0x1000 // 0x5000

// FrSky Sensor IDs
#define SENSOR_ID_VARIO        0x00 // Sensor ID  0
#define SENSOR_ID_FAS          0x22 // Sensor ID  2
#define SENSOR_ID_GPS          0x83 // Sensor ID  3
#define SENSOR_ID_SP2UR        0xC6 // Sensor ID  6
#define SENSOR_ID_28           0x1B // Sensor ID 28

class AP_Frsky_Telem
{
public:
    //constructor
    AP_Frsky_Telem(AP_AHRS &ahrs, const AP_BattMonitor &battery, const RangeFinder &rng, const AP_InertialNav_NavEKF &inav, const AC_Fence &fence, const AP_RPM &rpm_sensor, const AP_Motors &motors);

    struct msg_t // COULD BE PRIVATE??
    {
        mavlink_statustext_t data[MSG_BUFFER_LENGTH];
        uint8_t queued_idx;
        uint8_t sent_idx;
    };
    
    // init - perform required initialisation
    void init(const AP_SerialManager &serial_manager, const char *firmware_str, const char *frame_config_str, const uint8_t mav_type, AP_Float *fs_batt_voltage, AP_Float *fs_batt_mah, uint8_t *control_mode, uint32_t *ap_value, uint32_t *control_sensors_present, uint32_t *control_sensors_enabled, uint32_t *control_sensors_health, int32_t *home_distance, int32_t *home_bearing);
    void init(const AP_SerialManager &serial_manager, uint8_t *control_mode);
    // add statustext message to FrSky lib queue. This function is static so it can be called from any library.
    static void queue_message(MAV_SEVERITY severity, const char *text);

private:
    AP_AHRS &_ahrs;
    const AP_BattMonitor &_battery;
    const RangeFinder &_rng;
    const AP_InertialNav &_inav;
    const AC_Fence &_fence;
    const AP_RPM &_rpm_sensor;
    const AP_Motors &_motors;
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    uint16_t _crc;

    struct
    {
        uint8_t mav_type; // frame type (see MAV_TYPE in Mavlink definition file common.h)
        AP_Float *fs_batt_voltage; // failsafe battery voltage in volts
        AP_Float *fs_batt_mah; // failsafe reserve capacity in mAh
        // also a param: battery capacity in mAh as configured by user
    } _params;
    
    struct
    {
        uint8_t *control_mode;
        uint32_t *value;
        uint32_t *control_sensors_present;
        uint32_t *control_sensors_enabled;
        uint32_t *control_sensors_health;
        int32_t *home_distance;
        int32_t *home_bearing;
    } _ap;
    
    float _current_height; // in centimeters above home
    
    struct
    {
        char lat_ns, lon_ew;
        uint16_t latdddmm;
        uint16_t latmmmm;
        uint16_t londddmm;
        uint16_t lonmmmm;
        uint16_t alt_gps_meters;
        uint16_t alt_gps_cm;
        uint16_t alt_nav_meters;
        uint16_t alt_nav_cm;
        int16_t speed_in_meter;
        uint16_t speed_in_centimeter;
    } _gps;

    // main transmission function when protocol is FrSky SPort Passthrough (OpenTX)
    void send_SPort_Passthrough(void);
    // main transmission function when protocol is FrSky SPort
    void send_SPort(void);
    // main transmission function when protocol is FrSky D
    void send_D(void);
    // tick - main call to send updates to transmitter (called by scheduler at 1kHz)
    void tick(void);

    // methods related to the nuts-and-bolts of sending data
    void calc_crc(uint8_t byte);
    void send_crc(void);
    void send_byte(uint8_t value);
    void send_data(uint16_t id, uint32_t data);

    // methods to convert flight controller data to FrSky SPort Passthrough (OpenTX) format
    uint32_t get_next_msg_chunk(bool *chunk_sent);
    void control_sensors_check(void);
    void ekf_status_check(void);
    uint32_t calc_param(void);
    uint32_t calc_gps_latlng(bool *send_latitude);
    uint32_t calc_gps_status(void);
    uint32_t calc_batt(void);
    uint32_t calc_ap_status(void);
    uint32_t calc_home(void);
    uint32_t calc_velandrng(void); // calc_velandyaw
    uint32_t calc_attitude(void); // calc_attiandrng
    uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);

    // methods to convert flight controller data to FrSky D or SPort format
    void calc_nav_alt(void);
    float format_gps(float dec);
    void calc_gps_position(void);
};
