// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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

/* 
   FRSKY Telemetry library
*/
#include "AP_Frsky_Telem.h"
extern const AP_HAL::HAL& hal;

AP_Frsky_Telem::msg_t _msg;

//constructor
AP_Frsky_Telem::AP_Frsky_Telem(AP_AHRS &ahrs, const AP_BattMonitor &battery, const RangeFinder &rng, const AP_InertialNav_NavEKF &inav, const AC_Fence &fence, const AP_RPM &rpm_sensor, const AP_Motors &motors) :
    _ahrs(ahrs),
    _battery(battery),
    _rng(rng),
    _inav(inav),
    _fence(fence),
    _rpm_sensor(rpm_sensor),
    _motors(motors)
    {}

/*
 * init - perform required initialisation
 * for Copter
 */
void AP_Frsky_Telem::init(const AP_SerialManager &serial_manager, const char *firmware_str, const char *frame_config_str, const uint8_t mav_type, AP_Float *fs_batt_voltage, AP_Float *fs_batt_mah, uint8_t *control_mode, uint32_t *ap_value, uint32_t *control_sensors_present, uint32_t *control_sensors_enabled, uint32_t *control_sensors_health, int32_t *home_distance, int32_t *home_bearing)
{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_D, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_D; // FrSky D protocol (D-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort; // FrSky SPort protocol (X-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough; // FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
        // add firmware and frame info to message queue
        char text[50];
        strncpy(text, firmware_str, sizeof(text));
        strncat(text, " ", 1); // add a space before adding FRAME_CONFIG_STRING
        strncat(text, frame_config_str, 10); // FRAME_CONFIG_STRING shouldn't be more that 10 chars (see config.h)
        queue_message(MAV_SEVERITY_INFO, text);
        // save main parameters locally
        _params.mav_type = mav_type; // frame type (see MAV_TYPE in Mavlink definition file common.h)
        _params.fs_batt_voltage = fs_batt_voltage; // failsafe battery voltage in volts
        _params.fs_batt_mah = fs_batt_mah; // failsafe reserve capacity in mAh
        _ap.value = ap_value; // ap bit-field
        _ap.control_sensors_present = control_sensors_present;
        _ap.control_sensors_enabled = control_sensors_enabled;
        _ap.control_sensors_health = control_sensors_health;
        _ap.home_distance = home_distance;
        _ap.home_bearing = home_bearing;
    }

    _ap.control_mode = control_mode; // flight mode
    
    if (_port != NULL) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::tick, void));
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
}

/*
 * init - perform required initialisation
 * for Plane and Rover
 */
void AP_Frsky_Telem::init(const AP_SerialManager &serial_manager, uint8_t *control_mode)
{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_D, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_D; // FrSky D protocol (D-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort; // FrSky SPort protocol (X-receivers)
    }

    _ap.control_mode = control_mode;

    if (_port != NULL) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::tick, void));
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
}

/*
 * send telemetry data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort_Passthrough(void)
{
    static bool chunk_sent = true; // set to false whenever we are not done sending a chunk of mavlink message
    static bool send_attitude = false; // send_attiandrng
    static bool send_latitude = false;
    static uint32_t timer_params = 0;
    static uint32_t timer_ap_status = 0;
    static uint32_t timer_batt = 0;
    static uint32_t timer_gps_status = 0;
    static uint32_t timer_home = 0;
    static uint32_t timer_velandrng = 0; // timer_velandyaw
    static uint32_t timer_gps_latlng = 0;
    static uint32_t timer_vario = 0;
    static uint32_t timer_alt = 0;
    static uint32_t timer_vfas = 0;

    int16_t numc;
    numc = _port->available();

    // check if available is negative
    if (numc < 0) {
        return;
    }

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }

    // keep only the last two bytes of the data found in the serial buffer, as we shouldn't respond to old poll requests
    uint8_t prev_byte = 0;
    static uint8_t new_byte = 0;
    for (int16_t i = 0; i < numc; i++) {
        prev_byte = new_byte;
        new_byte = _port->read();
    }

    if ((prev_byte == START_STOP_SPORT) && (new_byte == SENSOR_ID_28)) { // byte 0x7E is the header of each poll request
        if (send_attitude) { // skip other data, send attitude // send_attiandrng // (roll, pitch) and range only this iteration
            send_attitude = false; // next iteration, check if we should send something other // send_attiandrng
        } else { // check if there's other data to send
            send_attitude = true; // next iteration, send attitude b/c it needs frequent updates to remain smooth // send_attiandrng
            // build mavlink message queue for control_sensors flags AND EKF_STATUSCHECK
            control_sensors_check();
            ekf_status_check();
            // if there's any message in the queue, start sending them chunk by chunk; three times each chunk
            if ((_msg.sent_idx != _msg.queued_idx) || (!chunk_sent)) {
                send_data(DIY_FIRST_ID, get_next_msg_chunk(&chunk_sent));
                return;
            }
            // send other sensor data if it's time for them, and reset the corresponding timer if sent
            uint32_t now = AP_HAL::millis();
            if (((now - timer_params) > 1000) && (!AP_Notify::flags.armed)) {
                send_data(DIY_FIRST_ID+7, calc_param());
                timer_params = AP_HAL::millis();
                return;
            } else if ((now - timer_ap_status) > 500) {
                send_data(DIY_FIRST_ID+1, calc_ap_status());
                timer_ap_status = AP_HAL::millis();
                return;
            } else if ((now - timer_batt) > 1000) {
                send_data(DIY_FIRST_ID+3, calc_batt());
                timer_batt = AP_HAL::millis();
                return;
            } else if ((now - timer_gps_status) > 1000) {
                send_data(DIY_FIRST_ID+2, calc_gps_status());
                timer_gps_status = AP_HAL::millis();
                return;
            } else if ((now - timer_home) > 500) {
                send_data(DIY_FIRST_ID+4, calc_home());
                timer_home = AP_HAL::millis();
                return;
            } else if ((now - timer_velandrng) > 500) { // timer_velandyaw
                send_data(DIY_FIRST_ID+5, calc_velandrng()); // calc_velandyaw
                timer_velandrng = AP_HAL::millis(); // timer_velandyaw
                return;
            } else if ((now - timer_gps_latlng) > 1000) {
                send_data(GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&send_latitude)); // gps latitude or longitude
                if (!send_latitude) { // we've cycled and sent one each of longitude then latitude, so reset the timer
                    timer_gps_latlng = AP_HAL::millis();
                }
                return;
            } else if ((now - timer_vario) > 500) {
                send_data(VARIO_FIRST_ID, (int32_t)roundf(_inav.get_velocity_z())); // vertical velocity in cm/s
                timer_vario = AP_HAL::millis();
                return;
            } else if ((now - timer_alt) > 1000) {
                send_data(ALT_FIRST_ID, (int32_t)_inav.get_altitude()); // altitude in cm above home position
                timer_alt = AP_HAL::millis();
                return;
            } else if ((now - timer_vfas) > 1000) {
                send_data(VFAS_FIRST_ID, (uint32_t)roundf(_battery.voltage() * 100.0f)); // battery pack voltage in volts
                timer_vfas = AP_HAL::millis();
                return;
            }
        }
        // if nothing else needed to be sent, send attitude // (roll, pitch) and range data
        send_data(DIY_FIRST_ID+6, calc_attitude()); // calc_attiandrng
    }
}

/*
 * send telemetry data
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort(void)
{
    static bool _sport_status = false;
    // variables keeping track of which data to send
    static uint8_t fas_call = 1;
    static uint8_t gps_call = 1;
    static uint8_t vario_call = 1;
    static uint8_t various_call = 1;

    int16_t numc;
    numc = _port->available();

    // check if available is negative
    if (numc < 0) {
        return;
    }

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }

    for (int16_t i = 0; i < numc; i++) {
        int16_t readbyte = _port->read();
        if (_sport_status == false) {
            if  (readbyte == START_STOP_SPORT) {
                _sport_status = true;
            }
        } else {
            switch(readbyte) {
                case SENSOR_ID_FAS:
                    switch (fas_call) {
                        case 1:
                            send_data(FUEL_ID, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
                            break;
                        case 2:
                            send_data(VFAS_ID, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
                            break;
                        case 3:
                            send_data(CURRENT_ID, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
                            break;
                    }
                    if (fas_call++ > 3) fas_call = 0;
                    break;
                case SENSOR_ID_GPS:
                    switch (gps_call) {
                        case 1:
                            calc_gps_position(); // gps data is not recalculated until all of it has been sent
                            send_data(GPS_LAT_BP_ID, _gps.latdddmm); // send gps lattitude degree and minute integer part
                            break;
                        case 2:
                            send_data(GPS_LAT_AP_ID, _gps.latmmmm); // send gps lattitude minutes decimal part
                            break;
                        case 3:
                            send_data(GPS_LAT_NS_ID, _gps.lat_ns); // send gps North / South information
                            break;
                        case 4:
                            send_data(GPS_LONG_BP_ID, _gps.londddmm); // send gps longitude degree and minute integer part
                            break;
                        case 5:
                            send_data(GPS_LONG_AP_ID, _gps.lonmmmm); // send gps longitude minutes decimal part
                            break;
                        case 6:
                            send_data(GPS_LONG_EW_ID, _gps.lon_ew); // send gps East / West information
                            break;
                        case 7:
                            send_data(GPS_SPEED_BP_ID, _gps.speed_in_meter); // send gps speed integer part
                            break;
                        case 8:
                            send_data(GPS_SPEED_AP_ID, _gps.speed_in_centimeter); // send gps speed decimal part
                            break;
                        case 9:
                            send_data(GPS_ALT_BP_ID, _gps.alt_gps_meters); // send gps altitude integer part
                            break;
                        case 10:
                            send_data(GPS_ALT_AP_ID, _gps.alt_gps_cm); // send gps altitude decimals
                            break;
                        case 11:
                            send_data(GPS_COURS_BP_ID, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
                            break;
                    }
                    if (gps_call++ > 11) gps_call = 1;
                    break;
                case SENSOR_ID_VARIO:
                    switch (vario_call) {
                        case 1 :
                            calc_nav_alt(); // nav altitude is not recalculated until all of it has been sent
                            send_data(BARO_ALT_BP_ID, _gps.alt_nav_meters); // send altitude integer part
                            break;
                        case 2:
                            send_data(BARO_ALT_AP_ID, _gps.alt_nav_cm); // send altitude decimal part
                            break;
                        }
                    if (vario_call++ > 2) vario_call = 1;
                    break;    
                case SENSOR_ID_SP2UR:
                    switch (various_call) {
                        case 1 :
                            send_data(TEMP2_ID, (uint16_t)(_ahrs.get_gps().num_sats() * 10 + _ahrs.get_gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                            break;
                        case 2:
                            send_data(TEMP1_ID, (uint16_t)*_ap.control_mode); // send flight mode
                            break;
                    }
                    if (various_call++ > 2) various_call = 1;
                    break;
            }
            _sport_status = false;
        }
    }
}

/*
 * send frame1 and frame2 telemetry data
 * one frame (frame1) is sent every 200ms with baro alt, nb sats, batt volts and amp, control_mode
 * a second frame (frame2) is sent every second (1000ms) with gps position data, and ahrs.yaw_sensor heading (instead of GPS heading)
 * for FrSky D protocol (D-receivers)
 */
void AP_Frsky_Telem::send_D(void)
{
    static uint32_t last_200ms_frame = 0;
    static uint32_t last_1000ms_frame = 0;

    uint32_t now = AP_HAL::millis();
    // send frame1 every 200ms
    if (now - last_200ms_frame > 200) {
        last_200ms_frame = now;
        send_data(TEMP2_ID, (uint16_t)(_ahrs.get_gps().num_sats() * 10 + _ahrs.get_gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
        send_data(TEMP1_ID, (uint16_t)*_ap.control_mode); // send flight mode
        send_data(FUEL_ID, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
        send_data(VFAS_ID, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
        send_data(CURRENT_ID, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
        calc_nav_alt();
        send_data(BARO_ALT_BP_ID, _gps.alt_nav_meters); // send nav altitude integer part
        send_data(BARO_ALT_AP_ID, _gps.alt_nav_cm); // send nav altitude decimal part
    }
    // send frame2 every second
    if (now - last_1000ms_frame > 1000) {
        last_1000ms_frame = now;
        send_data(GPS_COURS_BP_ID, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
        calc_gps_position();
        if (_ahrs.get_gps().status() >= 3) {
            send_data(GPS_LAT_BP_ID, _gps.latdddmm); // send gps lattitude degree and minute integer part
            send_data(GPS_LAT_AP_ID, _gps.latmmmm); // send gps lattitude minutes decimal part
            send_data(GPS_LAT_NS_ID, _gps.lat_ns); // send gps North / South information
            send_data(GPS_LONG_BP_ID, _gps.londddmm); // send gps longitude degree and minute integer part
            send_data(GPS_LONG_AP_ID, _gps.lonmmmm); // send gps longitude minutes decimal part
            send_data(GPS_LONG_EW_ID, _gps.lon_ew); // send gps East / West information
            send_data(GPS_SPEED_BP_ID, _gps.speed_in_meter); // send gps speed integer part
            send_data(GPS_SPEED_AP_ID, _gps.speed_in_centimeter); // send gps speed decimal part
            send_data(GPS_ALT_BP_ID, _gps.alt_gps_meters); // send gps altitude integer part
            send_data(GPS_ALT_AP_ID, _gps.alt_gps_cm); // send gps altitude decimal part
        }
    }
}

/*
 * tick - main call to send data to the receiver (called by scheduler at 1kHz)
 */
void AP_Frsky_Telem::tick(void)
{
    static bool initialised_uart = false; // true when we have detected the protocol and UART has been initialised
    // check UART has been initialised
    if (!initialised_uart) {
        // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
        if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                    // FrSky D protocol (D-receivers)
            _port->begin(AP_SERIALMANAGER_FRSKY_D_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        } else {                                                                        // FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
            _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        }
        initialised_uart = true;
    }

    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                        // FrSky D protocol (D-receivers)
        send_D();
    } else if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort) {             // FrSky SPort protocol (X-receivers)
        send_SPort();
    } else if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough) { // FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
        send_SPort_Passthrough();
    }
}

/* 
 * build up the frame's crc
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

/*
 * send the frame's crc at the end of the frame
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::send_crc(void) 
{
    send_byte(0xFF - _crc);
    _crc = 0;
}


/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_Telem::send_byte(uint8_t byte)
{
    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) { // FrSky D protocol (D-receivers)
        if (byte == 0x5E) {
            _port->write(0x5D);
            _port->write(0x3E);
        } else if (byte == 0x5D) {
            _port->write(0x5D);
            _port->write(0x3D);
        } else {
            _port->write(byte);
        }
    } else { // FrSky SPort protocol (X-receivers)
        if (byte == 0x7E) {
            _port->write(0x7D);
            _port->write(0x5E);
        } else if (byte == 0x7D) {
            _port->write(0x7D);
            _port->write(0x5D);
        } else {
            _port->write(byte);
        }
        calc_crc(byte);
    }
}

/*
 * send one frame of FrSky data
 */
void  AP_Frsky_Telem::send_data(uint16_t id, uint32_t data)
{
    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) { // FrSky D protocol (D-receivers)
        _port->write(0x5E);    // send a 0x5E start byte
        uint8_t *bytes = (uint8_t*)&id;
        send_byte(bytes[0]);
        bytes = (uint8_t*)&data;
        send_byte(bytes[0]); // LSB
        send_byte(bytes[1]); // MSB
    } else { // FrSky SPort protocol (X-receivers)
        send_byte(DATA_FRAME);
        uint8_t *bytes = (uint8_t*)&id;
        send_byte(bytes[0]); // LSB
        send_byte(bytes[1]); // MSB
        bytes = (uint8_t*)&data;
        send_byte(bytes[0]); // LSB
        send_byte(bytes[1]);
        send_byte(bytes[2]);
        send_byte(bytes[3]); // MSB
        send_crc();
    }
}

/*
 * grabs one "chunk" (4 bytes) of the mavlink statustext message to be transmitted
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::get_next_msg_chunk(bool *chunk_sent)
{
    static uint32_t chunk = 0; // a "chunk" (four characters/bytes) at a time of the mavlink message to be sent
    static uint8_t repeats = 3; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
    static uint8_t char_index = 0; // index of which character to get in the message
    
    if (repeats == 3) {
        chunk = 0;
        uint8_t character = _msg.data[_msg.sent_idx].text[char_index++];
        if (character) {
            chunk |= character<<24;
            character = _msg.data[_msg.sent_idx].text[char_index++];
            if (character) {
                chunk |= character<<16;
                character = _msg.data[_msg.sent_idx].text[char_index++];
                if (character) {
                    chunk |= character<<8;
                    character = _msg.data[_msg.sent_idx].text[char_index++];
                    if (character) {
                        chunk |= character;
                    }
                }
            }
        }
        if (!character) { // we've reached the end of the message (string terminated by '\0')
            char_index = 0;
            // add severity which is sent as the MSB of the last three bytes of the last chunk (bits 24, 16, and 8) since a character is on 7 bits
            chunk |= (_msg.data[_msg.sent_idx].severity & 0x4)<<21;
            chunk |= (_msg.data[_msg.sent_idx].severity & 0x2)<<14;
            chunk |= (_msg.data[_msg.sent_idx].severity & 0x1)<<7;
            _msg.sent_idx = (_msg.sent_idx + 1) % MSG_BUFFER_LENGTH;
        }
    }
    repeats--;
    if (repeats <= 0) {
        repeats = 3;
        *chunk_sent = true;
    }
    return chunk;
}

/*
 * add message to message cue for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::queue_message(MAV_SEVERITY severity, const char *text)
{
    _msg.data[_msg.queued_idx].severity = severity;
    strncpy((char *)_msg.data[_msg.queued_idx].text, text, 50);
    _msg.queued_idx = (_msg.queued_idx + 1) % MSG_BUFFER_LENGTH;
}

/*
 * add control_sensors information to message cue, normally passed as sys_status mavlink messages to the GCS, for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::control_sensors_check(void)
{
    static uint32_t control_sensors_timer = 0;
    uint32_t now = AP_HAL::millis();

    if ((now - control_sensors_timer) > 5000) { // prevent repeating any system_status messages unless 5 seconds have passed
        uint32_t _control_sensors_flags = (*_ap.control_sensors_health ^ *_ap.control_sensors_enabled) & *_ap.control_sensors_present;
        // only one error is reported at a time (in order of preference). Same setup and displayed messages as Mission Planner.
        if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_GPS) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad GPS Health");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_GYRO) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Gyro Health");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_ACCEL) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Accel Health");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_MAG) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Compass Health");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Baro Health");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_LASER_POSITION) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad LiDAR Health");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad OptFlow Health");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_TERRAIN) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad or No Terrain Data");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_GEOFENCE) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Geofence Breach");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_AHRS) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad AHRS");
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "NO RC Receiver");
            control_sensors_timer = now;
        }
    }
}

/*
 * add ekf_status information to message cue, normally passed as ekf_status_report mavlink messages to the GCS, for transmission through FrSky link
 * (for XPort protocol)
 */
void AP_Frsky_Telem::ekf_status_check(void)
{
    static uint32_t ekf_status_timer = 0;
    mavlink_ekf_status_report_t ekf_status_report;
    _ahrs.get_ekf_status_report(ekf_status_report);

    uint32_t now = AP_HAL::millis();
    if ((now - ekf_status_timer) > 10000) { // prevent repeating any ekf_status_report messages unless 10 seconds have passed
        // multiple errors can be reported at a time. Same setup as Mission Planner.
        if (ekf_status_report.velocity_variance >= 1) {
            queue_message(MAV_SEVERITY_CRITICAL, "Error velocity variance");
            ekf_status_timer = now;
        }

        if (ekf_status_report.pos_horiz_variance >= 1) {
            queue_message(MAV_SEVERITY_CRITICAL, "Error pos horiz variance");
            ekf_status_timer = now;
        }

        if (ekf_status_report.pos_vert_variance >= 1) {
            queue_message(MAV_SEVERITY_CRITICAL, "Error pos vert variance");
            ekf_status_timer = now;
        }

        if (ekf_status_report.compass_variance >= 1) {
            queue_message(MAV_SEVERITY_CRITICAL, "Error compass variance");
            ekf_status_timer = now;
        }

        if (ekf_status_report.terrain_alt_variance >= 1) {
            queue_message(MAV_SEVERITY_CRITICAL, "Error terrain alt variance");
            ekf_status_timer = now;
        }
    }
    /*
    flags found in AP_NavEKF/AP_Nav_Common.h | the flags are not used in Misison Planner yet
    ekf_status_report.flags.attitude                // 0 - true if attitude estimate is valid
    ekf_status_report.flags.horiz_vel                // 1 - true if horizontal velocity estimate is valid
    ekf_status_report.flags.vert_vel                // 2 - true if the vertical velocity estimate is valid
    ekf_status_report.flags.horiz_pos_rel            // 3 - true if the relative horizontal position estimate is valid
    ekf_status_report.flags.horiz_pos_abs            // 4 - true if the absolute horizontal position estimate is valid
    ekf_status_report.flags.vert_pos                // 5 - true if the vertical position estimate is valid
    ekf_status_report.flags.terrain_alt                // 6 - true if the terrain height estimate is valid
    ekf_status_report.flags.const_pos_mode            // 7 - true if we are in const position mode
    ekf_status_report.flags.pred_horiz_pos_rel        // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
    ekf_status_report.flags.pred_horiz_pos_abs        // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
    */
}

/*
 * prepare parameter data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_param(void)
{
    uint32_t param = 0; 
    static uint8_t paramID = 0;

    if (paramID >= 4) {
        paramID = 0;
    }
    paramID++;
    switch(paramID) {
    case 1:
        param = _params.mav_type;                                    // frame type (see MAV_TYPE in Mavlink definition file common.h)
        break;
    case 2:
        param = (uint32_t)roundf(*_params.fs_batt_voltage * 100.0f); // battery failsafe voltage in centivolts
        break;
    case 3:
        param = (uint32_t)roundf(*_params.fs_batt_mah);              // battery failsafe capacity in mAh
        break;
    case 4:
        param = (uint32_t)roundf(_battery.pack_capacity_mah());      // battery pack capacity in mAh as configured by user
        break;
    }
    //Reserve first 8 bits for param ID, use other 24 bits to store parameter value
    param = (paramID << 24) | (param & 0xFFFFFF);
    
    return param;
}

/*
 * prepare gps latitude/longitude data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_gps_latlng(bool *send_latitude)
{
    uint32_t latlng = 0;
    const Location &loc = _ahrs.get_gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    // alternate between latitude and longitude
    if (*send_latitude == true) {
        if (loc.lat < 0) {
            latlng |= ((abs(loc.lat)/100)*6) | 0x40000000;
        } else {
            latlng |= ((abs(loc.lat)/100)*6);
        }
        *send_latitude = false;
    } else {
        if (loc.lng < 0) {
            latlng |= ((abs(loc.lng)/100)*6) | 0xC0000000;
        } else {
            latlng |= ((abs(loc.lng)/100)*6) | 0x80000000;
        }
        *send_latitude = true;
    }
    return latlng;
}

/*
 * prepare gps status data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_gps_status(void)
{
    uint32_t gps_status = 0;

    // number of GPS satellites visible
    if (_ahrs.get_gps().num_sats() > 15) { // limit number of sats to 15
        gps_status |= 15;
    } else {
        gps_status |= _ahrs.get_gps().num_sats();
    }
    // GPS receiver status (i.e., NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D >= 3)
    if (_ahrs.get_gps().status() > 3) { // limit fix status to 3
        gps_status |= 3<<4; 
    } else {
        gps_status |= _ahrs.get_gps().status()<<4;
    }
    // GPS horizontal dilution of precision in dm
    gps_status |= prep_number(roundf(_ahrs.get_gps().get_hdop() * 0.1f),2,1)<<6; 
    // GPS vertical dilution of precision in dm
    gps_status |= prep_number(roundf(_ahrs.get_gps().get_vdop() * 0.1f),2,1)<<14; 
    // Altitude MSL in dm
    Location loc = _ahrs.get_gps().location();
    gps_status |= prep_number(roundf(loc.alt * 0.1f),2,2)<<22; 
    return gps_status;
}

/*
 * prepare battery data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_batt(void)
{
    uint32_t batt = 0;

    batt |= (((uint16_t)roundf(_battery.voltage() * 10.0f)) & 0x1FF); // battery voltage in decivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
    batt |= prep_number(roundf(_battery.current_amps() * 10.0f), 2, 1)<<9; // battery current draw in deciamps
    // battery current draw since power on in mAh
    if (_battery.current_total_mah() > 32767) { // limit to 32767 if above since value is encoded on 15 bits...
        batt |= 0x7FFF<<17;
    } else {
        batt |= (((uint16_t)roundf(_battery.current_total_mah())) & 0x7FFF)<<17;
    }
    return batt;
}

/*
 * prepare various ardupilot status data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_ap_status(void)
{
    uint32_t ap_status = 0;

    ap_status |= (uint8_t)(*_ap.control_mode & 0x1F); // flight mode number
    ap_status |= (uint8_t)(*_ap.value & 0x6)<<4; // simple/super simple modes flags
    ap_status |= (uint8_t)(*_ap.value & 0x80); // land complete flag
    ap_status |= (uint8_t)(AP_Notify::flags.armed)<<8; // armed flag
    ap_status |= (uint8_t)(AP_Notify::flags.failsafe_battery)<<9; // battery failsafe flag
    ap_status |= (uint8_t)(AP_Notify::flags.ekf_bad)<<10; // bad ekf flag
    ap_status |= (uint8_t)(_fence.get_breaches() & 0x3)<<11; // fence breach flags (1:alt breach, 2:circular breach, 3:both)
    // 4 bit gap here (reserved)
    ap_status |= prep_number(roundf(_rpm_sensor.get_rpm(0) * 0.01f), 2, 1)<<17; // 100s of RPM
    ap_status |= (((uint8_t)roundf(_motors.get_throttle() * 0.1f)) & 0x7F)<<25; // throttle %
    return ap_status;
}

/*
 * prepare home position related data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_home(void)
{
    uint32_t home = 0;

    // distance between vehicle and home location in meters
    home |= prep_number(roundf(*_ap.home_distance * 0.01f), 3, 2);
    // altitude between vehicle and home location in decimeters
    home |= prep_number(roundf(_inav.get_altitude() * 0.1f), 3, 2)<<12;
    // angle between vehicle and home location (relative to North) in 3 degree increments
    home |= (((uint8_t)roundf(*_ap.home_bearing * 0.00333f)) & 0x7F)<<25;
    return home;
}

/*
 * prepare velocity and range data // yaw
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_velandrng(void) // calc_velandyaw
{
    uint32_t velandrng = 0; // velandyaw

    velandrng |= prep_number(roundf(_inav.get_velocity_z() * 0.1f), 2, 1); // vertical velocity in dm/s // velandyaw
    velandrng |= prep_number(roundf(_inav.get_velocity_xy() * 0.1f), 2, 1)<<9; // horizontal velocity in dm/s // velandyaw
    velandrng |= prep_number(_rng.distance_cm(), 3, 1)<<17; // rangefinder measurement in cm // attiandrng <<21
    velandrng |= _rng.status()<<28; // rangefinder status (0:NotConnected,1:NoData,2:_OutOfRangeLow,3:OutOfRangeHigh,4:Good)
    velandrng |= _rng.pre_arm_check()<<31; // prearm check flag (set to true if rangefinder prearm passed)
    return velandrng;
}

/*
 * prepare attitude // (roll, pitch) and range data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_attitude(void) // calc_attiandrng
{
    uint32_t attitude = 0; // attiandrng

    attitude = ((uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & 0x7FF); // roll from [-18000;18000] centidegrees to unsigned .2 degree increments // attiandrng
    attitude |= ((uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f) & 0x3FF)<<11; // pitch from [-18000;18000] centidegrees to unsigned .2 degree increments // attiandrng
    attitude |= ((uint16_t)roundf(_ahrs.yaw_sensor * 0.05f) & 0x7FF)<<21; // yaw from [0;36000] centidegrees to .2 degree increments (already unsigned) // velandyaw <<17
    return attitude;
}

/*
 * prepare value for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint16_t AP_Frsky_Telem::prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

    if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = abs_number<<1;
        } else if (abs_number < 1270) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0xFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<8;
        }
    } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = abs_number<<2;
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 127000) {
            res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x1FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<9;
        }
    } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<1;
        } else if (abs_number < 10240) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x3FF x 10^1 = 10240)
            res = 0x7FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<11;
        }
    } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<2;
        } else if (abs_number < 10000) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 100000) {
            res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 1024000) {
            res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x3FF x 10^3 = 127000)
            res = 0xFFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<12;
        }
    }
    return res;
}

/*
 * prepare altitude between vehicle and home location data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Telem::calc_nav_alt(void)
{
    _gps.alt_nav_meters = (int16_t)_inav.get_altitude();
    _gps.alt_nav_cm = (_inav.get_altitude() - _gps.alt_nav_meters) * 100;
} 

/*
 * format the decimal latitude/longitude to the required degrees/minutes
 * for FrSky D and SPort protocols
 */
float AP_Frsky_Telem::format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare gps data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Telem::calc_gps_position(void)
{
    float lat;
    float lon;
    float alt;
    float speed;

    if (_ahrs.get_gps().status() >= 3) {
        Location loc = _ahrs.get_gps().location(); //get gps instance 0
        lat = format_gps(fabsf(loc.lat/10000000.0f));
        _gps.latdddmm = lat;
        _gps.latmmmm = (lat - _gps.latdddmm) * 10000;
        _gps.lat_ns = (loc.lat < 0) ? 'S' : 'N';

        lon = format_gps(fabsf(loc.lng/10000000.0f));
        _gps.londddmm = lon;
        _gps.lonmmmm = (lon - _gps.londddmm) * 10000;
        _gps.lon_ew = (loc.lng < 0) ? 'W' : 'E';

        alt = loc.alt * 0.01f;
        _gps.alt_gps_meters = (int16_t)alt;
        _gps.alt_gps_cm = (alt - _gps.alt_gps_meters) * 100;

        speed = _ahrs.get_gps().ground_speed();
        _gps.speed_in_meter = speed;
        _gps.speed_in_centimeter = (speed - _gps.speed_in_meter) * 100;
    } else {
        _gps.latdddmm = 0;
        _gps.latmmmm = 0;
        _gps.lat_ns = 0;
        _gps.londddmm = 0;
        _gps.lonmmmm = 0;
        _gps.alt_gps_meters = 0;
        _gps.alt_gps_cm = 0;
        _gps.speed_in_meter = 0;
        _gps.speed_in_centimeter = 0;
    }
}
