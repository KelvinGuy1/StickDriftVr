#include "joycon_read.h"
#include "openvr_driver.h"
#include "driverlog.h"
#include <string.h>

extern "C"{
    #include <mcu.h>
    #include <rumble.h>
    #include <input_report.h>
    #include <imu.h>
    #include <packet.h>
};

//#include "connect_controller.h"


#include "connect_controller.h"

#include "quaternion.h"

#include "imu_decode.h"

#include "attitude_estimator/attitude_estimator.h"

Joycon leftJoycon;
Joycon rightJoycon;
HIDController controllers;

float gyro[3];

//raw values for gyro
float offset_roll;
float offset_pitch;
float offset_yaw;
//raw values for accelerometer
float offset_posx;
float offset_posy;
float offset_posz;

//Button inputs for joycons
bool button_down = false,
    button_up = false,
    button_right = false,
    button_left = false,
    button_l = false,
    button_zl = false,
    button_sl_left = false,
    button_sr_left = false,
    button_minus = false,
    button_plus = false,
    button_rstick = false,
    button_lstick = false,
    button_home = false,
    button_capture = false,
    button_grip = false,
    button_y = false,
    button_x = false,
    button_b = false,
    button_a = false,
    button_r = false,
    button_zr = false,
    button_sl_right = false,
    button_sr_right = false;

uint8_t battery_status;

uint8_t timer = 0;

uint16_t calibrate_lx,
        calibrate_ly,
        calibrate_lx_min,
        calibrate_lx_max,
        calibrate_ly_min,
        calibrate_ly_max;

//Calibration values for right stick
uint16_t calibrate_rx,
        calibrate_ry,
        calibrate_rx_min,
        calibrate_rx_max,
        calibrate_ry_min,
        calibrate_ry_max;

//Stick inputs for joycons
int16_t stick_l_horizontal = 0;
int16_t stick_l_vertical = 0;
int16_t stick_r_horizontal = 0;
int16_t stick_r_vertical = 0;

int16_t
        acc_orgin_lx, acc_orgin_ly, acc_orgin_lz,
        acc_coeff_lx, acc_coeff_ly, acc_coeff_lz,
        gyro_orgin_lx, gyro_orgin_ly, gyro_orgin_lz,
        gyro_coeff_lx, gyro_coeff_ly, gyro_coeff_lz,

        acc_orgin_rx, acc_orgin_ry, acc_orgin_rz,
        acc_coeff_rx, acc_coeff_ry, acc_coeff_rz,
        gyro_orgin_rx, gyro_orgin_ry, gyro_orgin_rz,
        gyro_coeff_rx, gyro_coeff_ry, gyro_coeff_rz,

        roll_coeff, pitch_coeff, yaw_coeff,
        posx_coeff, posy_coeff, posz_coeff;


float left_P[4][4] = {
    {1e-3f, 0,     0,     0},
    {0,     1e-3f, 0,     0},
    {0,     0,     1e-3f, 0},
    {0,     0,     0,     1e-3f}
};

float right_P[4][4] = {
    {1e-3f, 0,     0,     0},
    {0,     1e-3f, 0,     0},
    {0,     0,     1e-3f, 0},
    {0,     0,     0,     1e-3f}
};


float imu_quaternion[4];

int16_t stick_rescale(int16_t value, int16_t min, int16_t max)
{
    if (max == min) return 0;

    // Clamp value to calibration range
    if (value < min) value = min;
    if (value > max) value = max;

    // Map value to [-32768, 32767]
    float norm = (float)(value - min) / (float)(max - min);
    int32_t scaled = (int32_t)(norm * 65535.0f - 32768.0f);

    // Clamp to int16_t range
    if (scaled < -32768) scaled = -32768;
    if (scaled > 32767) scaled = 32767;

    return (int16_t)scaled;
}

uint16_t joycon_input_report_right_stick_decode1(struct SIRStickStatus status) {
    return ((status.share) >> 4) | (status.vertical_high << 4);
}
// Extract button states using bitmasks

struct StickStatus joycon_input_report_stick_decode1(struct SIRStickStatus status) {
    struct StickStatus result = {
        .horizontal = joycon_input_report_left_stick_decode(status),
        .vertical = joycon_input_report_right_stick_decode1(status)
    };
    return result;
}

struct SubcommandBodySPIData joycon_spi_read_cal_stick_l()
{
    struct SubcommandBodySPIData spi_data = {
        .offset = SPI_REGION_CONF + 0x3D,
        .size = 0x12
    };
    return spi_data;
}

void joycon_packet_read_spi_cal_stick_l(uint8_t *buffer, uint8_t timer)
{
    memset(buffer, 0, OUTPUT_REPORT_LEGNTH);
    struct Header *hdr = (struct Header *)buffer;
    struct SubcommandBody *pkt = (struct SubcommandBody *)(hdr + 1);
    hdr->command = Subcommand;
    hdr->counter = timer;
    pkt->subcommand = SPIFlashRead;
    pkt->spi_data = joycon_spi_read_cal_stick_l();
}

bool get_stick_calibration(hid_device *handle, uint8_t timer, bool left_right) {
    uint8_t buf[OUTPUT_REPORT_LEGNTH];     // Buffer for an output packet

    timer ++;
    uint8_t buf_read[INPUT_REPORT_STANDARD_LEGNTH];  // Buffer for an input packet
    joycon_packet_read_spi_cal_stick_l(buf, timer & 0xF);
    hid_write(handle, buf, sizeof(buf));


    int res = hid_read(handle, buf_read, sizeof(buf_read));

    if (buf_read[0] != 0x21 || buf_read[13] != 0x90) {
        return false;
    }

    uint16_t data[11];
    data[0] = (buf_read[21] << 8) & 0xF00 | buf_read[20];
    data[1] = (buf_read[22] << 4) | (buf_read[21] >> 4);
    data[2] = (buf_read[24] << 8) & 0xF00 | buf_read[23];
    data[3] = (buf_read[25] << 4) | (buf_read[24] >> 4);
    data[4] = (buf_read[27] << 8) & 0xF00 | buf_read[26];
    data[5] = (buf_read[28] << 4) | (buf_read[27] >> 4);
    data[6] = (buf_read[30] << 8) & 0xF00 | buf_read[29];
    data[7] = (buf_read[31] << 4) | (buf_read[30] >> 4);
    data[8] = (buf_read[33] << 8) & 0xF00 | buf_read[32];
    data[9] = (buf_read[34] << 4) | (buf_read[33] >> 4);
    data[10] = (buf_read[36] << 8) & 0xF00 | buf_read[35];
    data[11] = (buf_read[37] << 4) | (buf_read[36] >> 4);

    if (left_right)
    {
        calibrate_lx = data[2];
        calibrate_ly = data[3];
        calibrate_lx_min = data[0];
        calibrate_lx_max = data[4];
        calibrate_ly_min = data[1];
        calibrate_ly_max = data[5];
    }
    else
    {
        calibrate_rx = data[6];
        calibrate_ry = data[7];
        calibrate_rx_min = data[8];
        calibrate_rx_max = data[10];
        calibrate_ry_min = data[9];
        calibrate_ry_max = data[11];
    }

    return true;
}

struct SubcommandBodySPIData joycon_spi_read_cal_gyro()
{
    struct SubcommandBodySPIData spi_data = {
        .offset = 0x8000 + 0x28,
        .size = 0x17
    };
    return spi_data;
}

void joycon_packet_spi_calIMU (uint8_t *buffer, uint8_t timer)
{
    memset(buffer, 0, OUTPUT_REPORT_LEGNTH);
    struct Header *hdr = (struct Header *)buffer;
    struct SubcommandBody *pkt = (struct SubcommandBody *)(hdr + 1);
    hdr->command = Subcommand;
    hdr->counter = timer;
    pkt->subcommand = SPIFlashRead;
    pkt->spi_data = joycon_spi_read_cal_gyro();
}

int16_t uint16_to_int16(uint16_t a) {
	int16_t b;
	char* aPointer = (char*)&a, *bPointer = (char*)&b;
	memcpy(bPointer, aPointer, sizeof(a));
	return b;
}

/*float acc_coeff_l [3];
float gyro_coeff_l [3];
float acc_coeff_r [3];
float gyro_coeff_r [3];*/

bool get_IMU_calibration(hid_device *handle, uint8_t timer, bool left_right) {
    uint8_t buf[OUTPUT_REPORT_LEGNTH];     // Buffer for an output packet

    timer ++;
    uint8_t buf_read[INPUT_REPORT_STANDARD_LEGNTH];  // Buffer for an input packet
    joycon_packet_spi_calIMU(buf, timer & 0xF);
    hid_write(handle, buf, sizeof(buf));


    int res = hid_read(handle, buf_read, sizeof(buf_read));

    if (buf_read[0] != 0x21 || buf_read[13] != 0x90) {
        return false;
    }

    int16_t data[24];
    data[0] = (buf_read[21] << 8) | buf_read[20];
    data[1] = (buf_read[23] << 8) | buf_read[22];
    data[2] = (buf_read[25] << 8) | buf_read[24];
    data[3] = (buf_read[27] << 8) | buf_read[26];
    data[4] = (buf_read[29] << 8) | buf_read[28];
    data[5] = (buf_read[31] << 8) | buf_read[30];
    data[6] = (buf_read[33] << 8) | buf_read[32];
    data[7] = (buf_read[35] << 8) | buf_read[34];
    data[8] = (buf_read[37] << 8) | buf_read[36];
    data[9] = (buf_read[39] << 8) | buf_read[38];
    data[10] = (buf_read[41] << 8) | buf_read[40];
    data[11] = (buf_read[43] << 8) | buf_read[42];

    if (left_right)
    {
        acc_orgin_lx = data[0];
        acc_orgin_ly = data[1];
        acc_orgin_lz = data[2];
        acc_coeff_lx = data[3];
        acc_coeff_ly = data[4];
        acc_coeff_lz = data[5];
        gyro_orgin_lx = data[6];
        gyro_orgin_ly = data[7];
        gyro_orgin_lz = data[8];
        gyro_coeff_lx = data[9];
        gyro_coeff_ly = data[10];
        gyro_coeff_lz = data[11];

        /*acc_coeff_l [0] = (float)(1.0 / (float)(acc_coeff_lx - uint16_to_int16(acc_orgin_lx))) * 4.0f;
        acc_coeff_l [1] = (float)(1.0 / (float)(acc_coeff_ly - uint16_to_int16(acc_orgin_ly))) * 4.0f;
        acc_coeff_l [2] = (float)(1.0 / (float)(acc_coeff_lz - uint16_to_int16(acc_orgin_lz))) * 4.0f;
        gyro_coeff_l [0] = (float)(816.0 / (float)(gyro_coeff_lx - uint16_to_int16(gyro_orgin_lx)));
        gyro_coeff_l [1] = (float)(816.0 / (float)(gyro_coeff_ly - uint16_to_int16(gyro_orgin_ly)));
        gyro_coeff_l [2] = (float)(816.0 / (float)(gyro_coeff_lz - uint16_to_int16(gyro_orgin_lz)));*/

    }
    else
    {
        acc_orgin_rx = data[0];
        acc_orgin_ry = data[1];
        acc_orgin_rz = data[2];
        acc_coeff_rx = data[3];
        acc_coeff_ry = data[4];
        acc_coeff_rz = data[5];
        gyro_orgin_rx = data[6];
        gyro_orgin_ry = data[7];
        gyro_orgin_rz = data[8];
        gyro_coeff_rx = data[9];
        gyro_coeff_ry = data[10];
        gyro_coeff_rz = data[11];

        /*acc_coeff_r [0] = (float)(1.0 / (float)(acc_coeff_rx - uint16_to_int16(acc_orgin_rx))) * 4.0f;
        acc_coeff_r [1] = (float)(1.0 / (float)(acc_coeff_ry - uint16_to_int16(acc_orgin_ry))) * 4.0f;
        acc_coeff_r [2] = (float)(1.0 / (float)(acc_coeff_rz - uint16_to_int16(acc_orgin_rz))) * 4.0f;
        gyro_coeff_r [0] = (float)(816.0 / (float)(gyro_coeff_rx - uint16_to_int16(gyro_orgin_rx)));
        gyro_coeff_r [1] = (float)(816.0 / (float)(gyro_coeff_ry - uint16_to_int16(gyro_orgin_ry)));
        gyro_coeff_r [2] = (float)(816.0 / (float)(gyro_coeff_rz - uint16_to_int16(gyro_orgin_rz)));*/
    }

    return true;
}

stateestimation::AttitudeEstimator left_estimator;
stateestimation::AttitudeEstimator right_estimator;

double left_q[4];
double right_q[4];

void init_attitude_estimator() {
    // Set PI gains and quick learning time as needed (tune for your hardware)
    left_estimator.setPIGains(2.2, 2.65, 10, 1.25);
    left_estimator.setQLTime(2.5);
    left_estimator.setAccMethod(left_estimator.ME_ABS_FUSED_YAW);
    right_estimator.setPIGains(2.2, 2.65, 10, 1.25);
    right_estimator.setQLTime(2.5);
    right_estimator.setAccMethod(right_estimator.ME_ABS_FUSED_YAW);
}

void read_imu_data(uint8_t *buf_read, int res,
                int16_t gyro_orgin[3], 
                int16_t gyro_coeff[3],
                int16_t acc_orgin[3], 
                int16_t acc_coeff[3])
{
    // Update angle
    struct InputReportHeader *header = (struct InputReportHeader *)buf_read;
    switch (header->id)
    {
        case SIRFullSpeed:
            //fprintf(stderr, "Received SIR:\t0x%02X, LEN: %d\n", header->id, res);

            // Ensure 'reply' is only initialized within this case and not before.
            {
                struct IMUPackedDataReply *reply = (struct IMUPackedDataReply *)(header + 1);

                enum IMUSensitivity Sensitive1 = Sensitivity3;
                enum IMUSensitivity Sensitive2 = Sensitivity3;

                // Process gyro data
                offset_roll = (0.005 * joycon_gyro_decode((reply->data[0].gyro_1.le_bytes.high << 8) | reply->data[0].gyro_1.le_bytes.low, Sensitive1))
                    +(0.005 * joycon_gyro_decode((reply->data[2].gyro_1.le_bytes.high << 8) | reply->data[2].gyro_1.le_bytes.low, Sensitive1))
                    +(0.005 * joycon_gyro_decode((reply->data[2].gyro_1.le_bytes.high << 8) | reply->data[2].gyro_1.le_bytes.low, Sensitive1));
                offset_pitch = (0.005 * joycon_gyro_decode((reply->data[0].gyro_2.le_bytes.high << 8) | reply->data[0].gyro_2.le_bytes.low, Sensitive1))
                    + (0.005 * joycon_gyro_decode((reply->data[1].gyro_2.le_bytes.high << 8) | reply->data[1].gyro_2.le_bytes.low, Sensitive1))
                    + (0.005 * joycon_gyro_decode((reply->data[2].gyro_2.le_bytes.high << 8) | reply->data[2].gyro_2.le_bytes.low, Sensitive1));
                offset_yaw = (0.005 * joycon_gyro_decode((reply->data[0].gyro_3.le_bytes.high << 8) | reply->data[0].gyro_3.le_bytes.low, Sensitive1))
                    + (0.005 * joycon_gyro_decode((reply->data[1].gyro_3.le_bytes.high << 8) | reply->data[1].gyro_3.le_bytes.low, Sensitive1))
                    + (0.005 * joycon_gyro_decode((reply->data[2].gyro_3.le_bytes.high << 8) | reply->data[2].gyro_3.le_bytes.low, Sensitive1));
                
                offset_posx = (0.005 * joycon_accel_decode((reply->data[0].accel_x.le_bytes.high << 8) | reply->data[0].accel_x.le_bytes.low, Sensitive2))
                    + (0.005 * joycon_accel_decode((reply->data[1].accel_x.le_bytes.high << 8) | reply->data[1].accel_x.le_bytes.low, Sensitive2))
                    + (0.005 * joycon_accel_decode((reply->data[2].accel_x.le_bytes.high << 8) | reply->data[2].accel_x.le_bytes.low, Sensitive2));
                offset_posy = (0.005 * joycon_accel_decode((reply->data[0].accel_y.le_bytes.high << 8) | reply->data[0].accel_y.le_bytes.low, Sensitive2))
                    + (0.005 * joycon_accel_decode((reply->data[1].accel_y.le_bytes.high << 8) | reply->data[1].accel_y.le_bytes.low, Sensitive2))
                    + (0.005 * joycon_accel_decode((reply->data[2].accel_y.le_bytes.high << 8) | reply->data[2].accel_y.le_bytes.low, Sensitive2));
                offset_posz = (0.005 * joycon_accel_decode((reply->data[0].accel_z.le_bytes.high << 8) | reply->data[0].accel_z.le_bytes.low, Sensitive2))
                    + (0.005 * joycon_accel_decode((reply->data[1].accel_z.le_bytes.high << 8) | reply->data[1].accel_z.le_bytes.low, Sensitive2))
                    + (0.005 * joycon_accel_decode((reply->data[2].accel_z.le_bytes.high << 8) | reply->data[2].accel_z.le_bytes.low, Sensitive2));

                roll_coeff = (float)(816.0 / (float)(gyro_coeff[0] - gyro_orgin[0]));
                offset_roll = (offset_roll - gyro_orgin[0]) * roll_coeff;
                pitch_coeff = (float)(816.0 / (gyro_coeff[1] - gyro_orgin[1]));
                offset_pitch = (offset_pitch - gyro_orgin[1]) * pitch_coeff;
                yaw_coeff = (float)(816.0 / (gyro_coeff[2] - gyro_orgin[2]));
                offset_yaw = (offset_yaw - gyro_orgin[2]) * yaw_coeff;

                /*posx_coeff = (float)(4.0 / (acc_coeff[0] - acc_orgin[0]));
                offset_posx = (offset_posx - acc_orgin[0])  * posx_coeff;
                posy_coeff = (float)(4.0 / (acc_coeff[1] - acc_orgin[1]));
                offset_posy = (offset_posy - acc_orgin[1])  * posy_coeff;
                posz_coeff = (float)(4.0 / (acc_coeff[2] - acc_orgin[2]));
                offset_posz = (offset_posz - acc_orgin[2]) * posz_coeff;

                offset_roll = (offset_roll + uint16_to_int16(gyro_orgin[0])) * gyro_coeff[0];
                offset_pitch = (offset_pitch + uint16_to_int16(gyro_orgin[1])) * gyro_coeff[1];
                offset_yaw = (offset_yaw + uint16_to_int16(gyro_orgin[2])) * gyro_coeff[2];
                offset_posx = (offset_posx + uint16_to_int16(acc_orgin[0])) * acc_coeff[0];
                offset_posy = (offset_posy + uint16_to_int16(acc_orgin[1])) * acc_coeff[1];
                offset_posz = (offset_posz + uint16_to_int16(acc_orgin[2])) * acc_coeff[2];*/
            }

            break; // Make sure to end this case with a break.

        default:
            //fprintf(stderr, "Received Other:\t0x%02X\n", header->id);
            break;
    }
}

void get_joycon_stick_input(uint8_t *buf_read, int res) {
    if (res < 0 || res < sizeof(InputReportHeader)) return;

    struct InputReportHeader *header = (struct InputReportHeader *)buf_read;

    StickStatus stick_status_l = joycon_input_report_stick_decode1(header->left_stick_status);
    StickStatus stick_status_r = joycon_input_report_stick_decode1(header->right_stick_status);

    // Defensive: Avoid division by zero
    int16_t lx_range = calibrate_lx_max - calibrate_lx_min;
    int16_t ly_range = calibrate_ly_max - calibrate_ly_min;
    int16_t rx_range = calibrate_rx_max - calibrate_rx_min;
    int16_t ry_range = calibrate_ry_max - calibrate_ry_min;

    if (lx_range == 0) lx_range = 1;
    if (ly_range == 0) ly_range = 1;
    if (rx_range == 0) rx_range = 1;
    if (ry_range == 0) ry_range = 1;

    stick_l_horizontal = stick_rescale((static_cast<int16_t>(stick_status_l.horizontal) - calibrate_lx), calibrate_lx_min, calibrate_lx_max);
    stick_l_vertical   = stick_rescale((static_cast<int16_t>(stick_status_l.vertical)   - calibrate_ly), calibrate_ly_min, calibrate_ly_max);
    stick_r_horizontal = stick_rescale((static_cast<int16_t>(stick_status_r.horizontal) - calibrate_rx), calibrate_rx_min, calibrate_rx_max);
    stick_r_vertical   = stick_rescale((static_cast<int16_t>(stick_status_r.vertical)   - calibrate_ry), calibrate_ry_min, calibrate_ry_max);
    battery_status = header->battery_level;
    
}

void get_joycon_input(uint8_t *buf_read, int res) {

    if (res < 0) {
        // Handle error: hid_read_timeout failed
        return;
    }

    if (res < sizeof(InputReportHeader)) {
        // Handle error: Not enough data read
        return;
    }

    struct InputReportHeader *header = (struct InputReportHeader *)buf_read;

    // Extract button states using bitmasks
    uint8_t btn_status_0 = header->btn_status.left;
    uint8_t btn_status_1 = header->btn_status.share;
    uint8_t btn_status_2 = header->btn_status.right;

    // Byte 1: Left Joy-Con
    button_down = (btn_status_0 & SIR_BUTTON_STATUS_MASK_DOWN) != 0;
    button_up = (btn_status_0 & SIR_BUTTON_STATUS_MASK_UP) != 0;
    button_right = (btn_status_0 & SIR_BUTTON_STATUS_MASK_RIGHT) != 0;
    button_left = (btn_status_0 & SIR_BUTTON_STATUS_MASK_LEFT) != 0;
    button_l = (btn_status_0 & SIR_BUTTON_STATUS_MASK_L) != 0;
    button_zl = (btn_status_0 & SIR_BUTTON_STATUS_MASK_ZL) != 0;
    button_sl_left = (btn_status_0 & SIR_BUTTON_STATUS_MASK_SL) != 0;
    button_sr_left = (btn_status_0 & SIR_BUTTON_STATUS_MASK_SR) != 0;

    // Byte 2: Shared buttons
    button_minus = (btn_status_1 & SIR_BUTTON_STATUS_MASK_MINUS) != 0;
    button_plus = (btn_status_1 & SIR_BUTTON_STATUS_MASK_PLUS) != 0;
    button_rstick = (btn_status_1 & SIR_BUTTON_STATUS_MASK_RSTICK) != 0;
    button_lstick = (btn_status_1 & SIR_BUTTON_STATUS_MASK_LSTICK) != 0;
    button_home = (btn_status_1 & SIR_BUTTON_STATUS_MASK_HOME) != 0;
    button_capture = (btn_status_1 & SIR_BUTTON_STATUS_MASK_CAPTURE) != 0;
    button_grip = (btn_status_1 & SIR_BUTTON_STATUS_MASK_GRIP) != 0;

    // Byte 3: Right Joy-Con
    button_y = (btn_status_2 & SIR_BUTTON_STATUS_MASK_Y) != 0;
    button_x = (btn_status_2 & SIR_BUTTON_STATUS_MASK_X) != 0;
    button_b = (btn_status_2 & SIR_BUTTON_STATUS_MASK_B) != 0;
    button_a = (btn_status_2 & SIR_BUTTON_STATUS_MASK_A) != 0;
    button_r = (btn_status_2 & SIR_BUTTON_STATUS_MASK_R) != 0;
    button_zr = (btn_status_2 & SIR_BUTTON_STATUS_MASK_ZR) != 0;
    button_sl_right = (btn_status_2 & SIR_BUTTON_STATUS_MASK_SL) != 0;
    button_sr_right = (btn_status_2 & SIR_BUTTON_STATUS_MASK_SR) != 0;
}

void Joycon::setVibration(float frequency) {
    
}


int res;

void update_joycon_r(double dt) {
    
    uint8_t buf_read_r[INPUT_REPORT_STANDARD_LEGNTH];

    res = hid_read(controllers.rightHIDHandle, buf_read_r, sizeof(buf_read_r));


    if (res <= 0) return;
    get_joycon_stick_input(buf_read_r, res);
    get_joycon_input(buf_read_r, res);

    int16_t gyro_orgin_arr[3] = {gyro_orgin_rx, gyro_orgin_ry, gyro_orgin_rz};
    int16_t gyro_coeff_arr[3] = {gyro_coeff_rx, gyro_coeff_ry, gyro_coeff_rz};
    int16_t acc_orgin_arr[3] = {acc_orgin_rx, acc_orgin_ry, acc_orgin_rz};
    int16_t acc_coeff_arr[3] = {acc_coeff_rx, acc_coeff_ry, acc_coeff_rz};
    read_imu_data(buf_read_r, res,
        gyro_orgin_arr, gyro_coeff_arr,
        acc_orgin_arr, acc_coeff_arr
    );
    //right_estimator.update(dt, ((offset_pitch - uint16_to_int16(gyro_coeff_ry)) * gyro_coeff_r[1]), -((offset_roll - uint16_to_int16(gyro_coeff_rx)) * gyro_coeff_r[0]), ((offset_yaw - uint16_to_int16(gyro_coeff_rz)) * gyro_coeff_r[2]), ((offset_posy - uint16_to_int16(acc_coeff_ry)) * acc_coeff_r[1]), -((offset_posx - uint16_to_int16(acc_coeff_rx)) * acc_coeff_r[0]), ((offset_posz - uint16_to_int16(acc_coeff_rz)) * acc_coeff_r[2]), 0, 0, 0);
    right_estimator.update(dt, offset_pitch, -offset_roll, offset_yaw, offset_posy, -offset_posx, offset_posz, 0, 0, 0);
    right_estimator.getAttitude(right_q);
    
    // Update controler states

    // Right joycon
    rightJoycon.a = button_a;
    rightJoycon.b = button_b;
    rightJoycon.x = button_x;
    rightJoycon.y = button_y;
    rightJoycon.shoulder = button_r;
    rightJoycon.z_shoulder = button_zr;
    rightJoycon.sl_bumper = button_sl_right;
    rightJoycon.sr_bumper = button_sr_right;
    rightJoycon.shutter_button = button_capture;
    rightJoycon.pause_button = button_plus;
    rightJoycon.stick_click = button_rstick;
    rightJoycon.stick_x = (float)stick_r_horizontal/32767.0f;
    rightJoycon.stick_y = (float)stick_r_vertical/32767.0f;
    rightJoycon.batteryLevel = battery_status;
    rightJoycon.imu_w = right_q[0];  // Update quaternion
    rightJoycon.imu_x = right_q[1];
    rightJoycon.imu_y = right_q[2];
    rightJoycon.imu_z = right_q[3];
}

void update_joycon_l(double dt) {

    uint8_t buf_read_l[INPUT_REPORT_STANDARD_LEGNTH];

    res = hid_read(controllers.leftHIDHandle, buf_read_l, sizeof(buf_read_l));


    if (res <= 0) return;
    get_joycon_stick_input(buf_read_l, res);
    get_joycon_input(buf_read_l, res);

    int16_t gyro_orgin_arr[3] = {gyro_orgin_lx, gyro_orgin_ly, gyro_orgin_lz};
    int16_t gyro_coeff_arr[3] = {gyro_coeff_lx, gyro_coeff_ly, gyro_coeff_lz};
    int16_t acc_orgin_arr[3] = {acc_orgin_lx, acc_orgin_ly, acc_orgin_lz};
    int16_t acc_coeff_arr[3] = {acc_coeff_lx, acc_coeff_ly, acc_coeff_lz};
    read_imu_data(buf_read_l, res,
        gyro_orgin_arr, gyro_coeff_arr,
        acc_orgin_arr, acc_coeff_arr
    );
    //left_estimator.update(dt, ((offset_pitch - uint16_to_int16(gyro_coeff_ly)) * gyro_coeff_l[1]), -((offset_roll - uint16_to_int16(gyro_coeff_lx)) * gyro_coeff_l[0]), -((offset_yaw - uint16_to_int16(gyro_coeff_lz)) * gyro_coeff_l[2]), -((offset_posy - uint16_to_int16(acc_coeff_ly)) * acc_coeff_l[1]), -((offset_posx - uint16_to_int16(acc_coeff_lx)) * acc_coeff_l[0]), -((offset_posz - uint16_to_int16(acc_coeff_lz)) * acc_coeff_l[2]), 0, 0, 0);
    left_estimator.update(dt, offset_pitch, -offset_roll, -offset_yaw, -offset_posy, -offset_posx, -offset_posz, 0, 0, 0);
    left_estimator.getAttitude(left_q);

    // Left joycon
    leftJoycon.a = button_right;
    leftJoycon.b = button_down;
    leftJoycon.x = button_up;
    leftJoycon.y = button_left;
    leftJoycon.shoulder = button_l;
    leftJoycon.z_shoulder = button_zl;
    leftJoycon.sl_bumper = button_sl_left;
    leftJoycon.sr_bumper = button_sr_left;
    leftJoycon.shutter_button = button_capture;
    leftJoycon.pause_button = button_minus;
    leftJoycon.stick_click = button_lstick;
    leftJoycon.stick_x = stick_l_horizontal/32768.0;
    leftJoycon.stick_y = stick_l_vertical/32768.0;
    leftJoycon.batteryLevel = battery_status;
    leftJoycon.imu_w = left_q[0];  // Update quaternion
    leftJoycon.imu_x = left_q[1];
    leftJoycon.imu_y = left_q[2];
    leftJoycon.imu_z = left_q[3];
    

    timer++;
}

bool joycon_init_left() {
    leftJoycon.isConnected = false;
    if (controllers.search_controller_left()) {
        vr::VRDriverLog()->Log("Got controllers (left)");
        uint8_t buf_l[OUTPUT_REPORT_LEGNTH];
        uint8_t timer_l = 0;
        uint8_t buf_read_l[INPUT_REPORT_STANDARD_LEGNTH];
        DriverLog("Mounting left");
        joycon_packet_input_report_mode(buf_l, timer_l & 0xF, 0x23);
        for (int i = 0; i < 20; i++) {
            hid_write(controllers.leftHIDHandle, buf_l, sizeof(buf_l));
        }
        timer_l++;

        joycon_packet_imu_enable(buf_l, timer_l & 0xF);
        for (int i = 0; i < 20; i++) {
            hid_write(controllers.leftHIDHandle, buf_l, sizeof(buf_l));
        }
        timer_l++;

        while (!get_stick_calibration(controllers.leftHIDHandle, timer_l & 0xF, true)) {
            timer_l++;
        }

        while (!get_IMU_calibration(controllers.leftHIDHandle, timer_l & 0xF, true)) {
            timer_l++;
        }

        while (buf_read_l[0] != 0x30) {
            joycon_packet_input_report_mode(buf_l, timer_l & 0xF, SIRFullSpeed);
            hid_write(controllers.leftHIDHandle, buf_l, sizeof(buf_l));
            timer_l++;
            hid_read(controllers.leftHIDHandle, buf_read_l, sizeof(buf_read_l));
        }

        leftJoycon.isConnected = true;
        DriverLog("Success (left)!");
    }
    return leftJoycon.isConnected;
}

bool joycon_init_right() {
    rightJoycon.isConnected = false;
    if (controllers.search_controller_right()) {
        vr::VRDriverLog()->Log("Got controllers (right)");
        uint8_t buf_r[OUTPUT_REPORT_LEGNTH];
        uint8_t timer_r = 0;
        uint8_t buf_read_r[INPUT_REPORT_STANDARD_LEGNTH];
        DriverLog("Mounting right");
        for (int i = 0; i < 20; i++) {
            joycon_packet_input_report_mode(buf_r, timer_r & 0xF, 0x23);
            hid_write(controllers.rightHIDHandle, buf_r, sizeof(buf_r));
            joycon_packet_imu_enable(buf_r, timer_r & 0xF);
            hid_write(controllers.rightHIDHandle, buf_r, sizeof(buf_r));
        }
        timer_r++;

        while (!get_stick_calibration(controllers.rightHIDHandle, timer_r & 0xF, false)) {
            timer_r++;
        }

        while (!get_IMU_calibration(controllers.rightHIDHandle, timer_r & 0xF, false)) {
            timer_r++;
        }

        while (buf_read_r[0] != 0x30) {
            joycon_packet_input_report_mode(buf_r, timer_r & 0xF, SIRFullSpeed);
            hid_write(controllers.rightHIDHandle, buf_r, sizeof(buf_r));
            timer_r++;
            hid_read(controllers.rightHIDHandle, buf_read_r, sizeof(buf_read_r));
        }

        rightJoycon.isConnected = true;
        DriverLog("Success (right)!");
    }
    return rightJoycon.isConnected;
}
// bool joycon_init() {
//     leftJoycon.isConnected = false;
//     rightJoycon.isConnected = false;
//     if (controllers.searchController()) {
//         vr::VRDriverLog()->Log( "Got controllers" );
//         uint8_t buf_l[OUTPUT_REPORT_LEGNTH];
//         uint8_t timer_l = 0;
//         int res_l;
//         uint8_t buf_read_l[INPUT_REPORT_STANDARD_LEGNTH];
//         uint8_t buf_r[OUTPUT_REPORT_LEGNTH];
//         uint8_t timer_r = 0;
//         int res_r;
//         uint8_t buf_read_r[INPUT_REPORT_STANDARD_LEGNTH];
//         DriverLog( "Mounting left" );
//         joycon_packet_input_report_mode(buf_l, timer_l & 0xF, 0x23);
//         for (int i = 0; i < 20; i++) {
//             hid_write(controllers.leftHIDHandle, buf_l, sizeof(buf_l));
//         }
//         timer_l++;

//         joycon_packet_imu_enable(buf_l, timer_l & 0xF);
//         for (int i = 0; i < 20; i++) {
//             hid_write(controllers.leftHIDHandle, buf_l, sizeof(buf_l));
//         }
//         timer_l++;
    
//         while (!get_stick_calibration(controllers.leftHIDHandle, timer_l & 0xF, true)) {
//             timer_l++;
//         }

//         while (!get_IMU_calibration(controllers.leftHIDHandle, timer_l & 0xF, true)) {
//             timer_l++;
//         }

//         while (buf_read_l[0] != 0x30) {
//             joycon_packet_input_report_mode(buf_l, timer_l & 0xF, SIRFullSpeed);
//             hid_write(controllers.leftHIDHandle, buf_l, sizeof(buf_l));
//             timer_l++;
//             hid_read(controllers.leftHIDHandle, buf_read_l, sizeof(buf_read_l));
//         }

//         leftJoycon.isConnected = true;
//         DriverLog( "Success!" );

//         DriverLog( "Mounting right" );

//         for (int i = 0; i < 20; i++) {
//             joycon_packet_input_report_mode(buf_r, timer_r & 0xF, 0x23);
//             hid_write(controllers.rightHIDHandle, buf_r, sizeof(buf_r));
//             joycon_packet_imu_enable(buf_r, timer_r & 0xF);
//             hid_write(controllers.rightHIDHandle, buf_r, sizeof(buf_r));
//         }
//         timer_r++;

//         while (!get_stick_calibration(controllers.rightHIDHandle, timer_r & 0xF, false)) {
//             timer_r++;
//         }

//         while (!get_IMU_calibration(controllers.rightHIDHandle, timer_r & 0xF, false)) {
//             timer_r++;
//         }

//         while (buf_read_r[0] != 0x30) {
//             joycon_packet_input_report_mode(buf_r, timer_r & 0xF, SIRFullSpeed);
//             hid_write(controllers.rightHIDHandle, buf_r, sizeof(buf_r));
//             timer_r++;
//             hid_read(controllers.rightHIDHandle, buf_read_r, sizeof(buf_read_r));
//         }

//         rightJoycon.isConnected = true;
//         DriverLog( "Success!" );

//         return true;
//     }

//     return false;
// }

void joycon_deinit() {
    controllers.freeControllers();
    leftJoycon.isConnected = false;
    rightJoycon.isConnected = false;
}
