//============ Copyright (c) Valve Corporation, All rights reserved. ============
#pragma once

#include <array>
#include <string>


#include "openvr_driver.h"
#include <atomic>
#include <thread>

#include "joycon_read.h"

enum MyComponent
{
	MyComponent_battery,
	MyComponent_a_touch,
	MyComponent_a_click,
	MyComponent_b_touch,
	MyComponent_b_click,
	MyComponent_x_touch,
	MyComponent_x_click,
	MyComponent_y_touch,
	MyComponent_y_click,
	MyComponent_plus_minus_click,
	MyComponent_capture_click,

	MyComponent_sl_click,
	MyComponent_sr_click,

	MyComponent_stick_click,
	MyComponent_stick_x,
	MyComponent_stick_y,

	MyComponent_trigger_z_value,
	MyComponent_trigger_z_click,

	MyComponent_trigger_value,
	MyComponent_trigger_click,

	MyComponent_haptic,

	MyComponent_MAX
};;

//-----------------------------------------------------------------------------
// Purpose: Represents a single tracked device in the system.
// What this device actually is (controller, hmd) depends on the
// properties you set within the device (see implementation of Activate)
//-----------------------------------------------------------------------------
class MyControllerDeviceDriver : public vr::ITrackedDeviceServerDriver
{
public:
	MyControllerDeviceDriver( vr::ETrackedControllerRole role );

	vr::EVRInitError Activate( uint32_t unObjectId ) override;

	void EnterStandby() override;

	void *GetComponent( const char *pchComponentNameAndVersion ) override;

	void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize ) override;

	vr::DriverPose_t GetPose() override;

	void update_joy_pose(vr::ETrackedControllerRole);

	void Deactivate() override;

	// ----- Functions we declare ourselves below -----

	const std::string &MyGetSerialNumber();

	void MyRunFrame();
	void MyProcessEvent( const vr::VREvent_t &vrevent );

	void MyPoseUpdateThread();	

	void MyPoseMountThread();

	vr::ETrackedControllerRole my_controller_role_;
	std::thread my_pose_update_thread_;

	std::chrono::steady_clock::time_point last_update_time_l;
	std::chrono::steady_clock::time_point last_update_time_r;

private:
	std::atomic< vr::TrackedDeviceIndex_t > my_controller_index_;

	std::string my_controller_model_number_;
	std::string my_controller_serial_number_;

	vr::VRInputComponentHandle_t input_handles_[MyComponent_MAX];

	std::atomic< bool > is_active_;
	std::thread my_pose_mount_thread_;
};