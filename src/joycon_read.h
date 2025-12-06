#ifndef JOYCON_READ_H
#define JOYCON_READ_H

class Joycon {
public:
    // Button states

    //hid_device left_handle;
    //hid_device right_handle;
    
    bool a;
    bool b;
    bool x;
    bool y;
    bool shoulder;
    bool z_shoulder;
    bool sl_bumper;
    bool sr_bumper;
    bool shutter_button;
    bool pause_button;
    bool stick_click;

    // Gyroscope and accelerometer data
    float imu_w = 1.0f; // Default to identity quaternion
    float imu_x = 0.0f;
    float imu_y = 0.0f;
    float imu_z = 0.0f;

    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    // Stick input
    float stick_x;
    float stick_y;

    // Connection status
    bool isConnected = false;

    // Battery level
    float batteryLevel;

    // Function to set vibrations
    void setVibration(float frequency);
};

extern Joycon leftJoycon;
extern Joycon rightJoycon;

// Update function to get the latest state of the controller
void update_joycon_l(double dt);
void update_joycon_r(double dt);

// Function to initialize the Joycon controllers
bool joycon_init_left();
bool joycon_init_right();

void joycon_deinit();


#endif // JOYCON_READ_H