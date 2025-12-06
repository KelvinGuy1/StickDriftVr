//============ Copyright (c) Valve Corporation, All rights reserved. ============
#include "controller_device_driver.h"

#include "driverlog.h"
#include "vrmath.h"

#include "joycon_read.h"

#include "joycon_read.h"

//#include <ivrvirtualdisplay.h>

// Let's create some variables for strings used in getting settings.
// This is the section where all of the settings we want are stored. A section name can be anything,
// but if you want to store driver specific settings, it's best to namespace the section with the driver identifier
// ie "<my_driver>_<section>" to avoid collisions
static const char *my_controller_main_settings_section = "driver_simplecontroller";

// Individual right/left hand settings sections
static const char *my_controller_right_settings_section = "driver_simplecontroller_left_controller";
static const char *my_controller_left_settings_section = "driver_simplecontroller_right_controller";

// These are the keys we want to retrieve the values for in the settings
static const char *my_controller_settings_key_model_number = "mycontroller_model_number";
static const char *my_controller_settings_key_serial_number = "mycontroller_serial_number";


MyControllerDeviceDriver::MyControllerDeviceDriver( vr::ETrackedControllerRole role )
{
	// Set a member to keep track of whether we've activated yet or not
	is_active_ = false;

	// The constructor takes a role argument, that gives us information about if our controller is a left or right hand.
	// Let's store it for later use. We'll need it.
	my_controller_role_ = role;

	// We have our model number and serial number stored in SteamVR settings. We need to get them and do so here.
	// Other IVRSettings methods (to get int32, floats, bools) return the data, instead of modifying, but strings are
	// different.
	char model_number[ 1024 ];
	vr::VRSettings()->GetString( my_controller_main_settings_section, my_controller_settings_key_model_number, model_number, sizeof( model_number ) );
	my_controller_model_number_ = model_number;

	// Get our serial number depending on our "handedness"
	char serial_number[ 1024 ];
	vr::VRSettings()->GetString( my_controller_role_ == vr::TrackedControllerRole_LeftHand ? my_controller_left_settings_section : my_controller_right_settings_section,
		my_controller_settings_key_serial_number, serial_number, sizeof( serial_number ) );
	my_controller_serial_number_ = serial_number;

	// Here's an example of how to use our logging wrapper around IVRDriverLog
	// In SteamVR logs (SteamVR Hamburger Menu > Developer Settings > Web console) drivers have a prefix of
	// "<driver_name>:". You can search this in the top search bar to find the info that you've logged.
	DriverLog( "My Controller Model Number: %s", my_controller_model_number_.c_str() );
	DriverLog( "My Controller Serial Number: %s", my_controller_serial_number_.c_str() );
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver after our
//  IServerTrackedDeviceProvider calls IVRServerDriverHost::TrackedDeviceAdded.
//-----------------------------------------------------------------------------
vr::EVRInitError MyControllerDeviceDriver::Activate( uint32_t unObjectId )
{
	// Set an member to keep track of whether we've activated yet or not
	is_active_ = true;
	last_update_time_l = std::chrono::steady_clock::now();
	last_update_time_r = std::chrono::steady_clock::now();

	// Let's keep track of our device index. It'll be useful later.
	my_controller_index_ = unObjectId;

	// Properties are stored in containers, usually one container per device index. We need to get this container to set
	// The properties we want, so we call this to retrieve a handle to it.
	vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer( my_controller_index_ );
	
	// Let's begin setting up the properties now we've got our container.
	// A list of properties available is contained in vr::ETrackedDeviceProperty.

	// First, let's set the model number.
	vr::VRProperties()->SetStringProperty( container, vr::Prop_ModelNumber_String, my_controller_model_number_.c_str() );
	vr::VRProperties()->SetBoolProperty( container, vr::Prop_WillDriftInYaw_Bool, true );

	// Let's tell SteamVR our role which we received from the constructor earlier.
	vr::VRProperties()->SetInt32Property( container, vr::Prop_ControllerRoleHint_Int32, my_controller_role_ );


	// Now let's set up our inputs

	// This tells the UI what to show the user for bindings for this controller,
	// As well as what default bindings should be for legacy apps.
	// Note, we can use the wildcard {<driver_name>} to match the root folder location
	// of our driver.
	vr::VRProperties()->SetStringProperty( container, vr::Prop_InputProfilePath_String, "{simplecontroller}/input/mycontroller_profile.json" );

	// Let's set up handles for all of our components.
	// Even though these are also defined in our input profile,
	// We need to get handles to them to update the inputs.
	// Initialize input handle for battery status
	//vr::VRDriverInput()->CreateScalarComponent(container, "/input/battery", &input_handles_[MyComponent_battery], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
	// Initialize input handles for joycon buttons

	// Initialize input handles for gyro support
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/a/click", &input_handles_[MyComponent_a_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/b/click", &input_handles_[MyComponent_b_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/x/click", &input_handles_[MyComponent_x_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/y/click", &input_handles_[MyComponent_y_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/plus_minus/click", &input_handles_[MyComponent_plus_minus_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/capture/click", &input_handles_[MyComponent_capture_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/sl/click", &input_handles_[MyComponent_sl_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/sr/click", &input_handles_[MyComponent_sr_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trigger/click", &input_handles_[MyComponent_trigger_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/z_trigger/click", &input_handles_[MyComponent_trigger_z_click]);
	vr::VRDriverInput()->CreateBooleanComponent(container, "/input/stick/click", &input_handles_[MyComponent_stick_click]);
	vr::VRDriverInput()->CreateScalarComponent(container, "/input/stick/x", &input_handles_[MyComponent_stick_x], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(container, "/input/stick/y", &input_handles_[MyComponent_stick_y], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

	// Let's create our haptic component.
	// These are global across the device, and you can only have one per device.
	vr::VRDriverInput()->CreateHapticComponent( container, "/output/haptic", &input_handles_[ MyComponent_haptic ] );

	my_pose_update_thread_ = std::thread( &MyControllerDeviceDriver::MyPoseUpdateThread, this );
	my_pose_mount_thread_ = std::thread( &MyControllerDeviceDriver::MyPoseMountThread, this );

	// We've activated everything successfully!
	// Let's tell SteamVR that by saying we don't have any errors.
	return vr::VRInitError_None;
}

//-----------------------------------------------------------------------------
// Purpose: If you're an HMD, this is where you would return an implementation
// of vr::IVRDisplayComponent, vr::IVRVirtualDisplay or vr::IVRDirectModeComponent.
//
// But this a simple example to demo for a controller, so we'll just return nullptr here.
//-----------------------------------------------------------------------------
void *MyControllerDeviceDriver::GetComponent( const char *pchComponentNameAndVersion )
{
	return nullptr;
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver when a debug request has been made from an application to the driver.
// What is in the response and request is up to the application and driver to figure out themselves.
//-----------------------------------------------------------------------------
void MyControllerDeviceDriver::DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
	if ( unResponseBufferSize >= 1 ){
		pchResponseBuffer[ 0 ] = 0;
	}
}


//-----------------------------------------------------------------------------
// Purpose: This is never called by vrserver in recent OpenVR versions,
// but is useful for giving data to vr::VRServerDriverHost::TrackedDevicePoseUpdated.
//-----------------------------------------------------------------------------
vr::DriverPose_t MyControllerDeviceDriver::GetPose(){
	vr::DriverPose_t pose = { 0 };
	return pose;
}

bool IsHMDTracking()
{
    vr::TrackedDevicePose_t hmd_pose{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &hmd_pose, 1);

    // Check if the HMD pose is valid
    return hmd_pose.bPoseIsValid && hmd_pose.bDeviceIsConnected;
}

void MyControllerDeviceDriver::update_joy_pose(vr::ETrackedControllerRole role)
{
	// Let's retrieve the Hmd pose to base our controller pose off.

	// First, initialize the struct that we'll be submitting to the runtime to tell it we've updated our pose.
	vr::DriverPose_t pose = { 0 };

	pose.poseTimeOffset = 0.0f;

	pose.qWorldFromDriverRotation.w = 1.f;
	pose.qDriverFromHeadRotation.w = 1.f;

	vr::TrackedDevicePose_t hmd_pose{};

	// GetRawTrackedDevicePoses expects an array.
	// We only want the hmd pose, which is at index 0 of the array so we can just pass the struct in directly, instead of in an array
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses( 0, &hmd_pose, 1 );

	// Get the position of the hmd from the 3x4 matrix GetRawTrackedDevicePoses returns
	vr::HmdVector3_t hmd_position = HmdVector3_From34Matrix( hmd_pose.mDeviceToAbsoluteTracking );
	// Get the orientation of the hmd from the 3x4 matrix GetRawTrackedDevicePoses returns
	vr::HmdQuaternion_t hmd_orientation = HmdQuaternion_FromMatrix( hmd_pose.mDeviceToAbsoluteTracking );

	vr::HmdVector3_t offset_position = {
		role == vr::TrackedControllerRole_LeftHand ? -0.20f : 0.20f,
		-0.1f,
		-0.4f,
	};

	// Rotate our offset by the hmd quaternion (so the controllers are always facing towards us), and add then add the position of the hmd to put it into position.
	vr::HmdVector3_t position = hmd_position + ( offset_position * hmd_orientation );

	// Get the yaw quaternion from the hmd orientation
	double hmd_yaw = std::atan2(hmd_position.v[0] - position.v[0], hmd_position.v[2] - position.v[2]);

	// Use the extracted yaw to create the offset orientation
	vr::HmdQuaternion_t offset_orientation;
	//hmd_yaw + DEG_TO_RAD(-leftJoycon.gyro_x)
	if (role == vr::TrackedControllerRole_LeftHand) { offset_orientation.w = leftJoycon.imu_w; offset_orientation.x = leftJoycon.imu_x; offset_orientation.y = leftJoycon.imu_y; offset_orientation.z = leftJoycon.imu_z; } else { offset_orientation.w = rightJoycon.imu_w; offset_orientation.x = rightJoycon.imu_x; offset_orientation.y = rightJoycon.imu_y; offset_orientation.z = rightJoycon.imu_z; }

	//if (role == vr::TrackedControllerRole_LeftHand) { offset_orientation = HmdQuaternion_FromEulerAngles(DEG_TO_RAD(-leftJoycon.gyro_x), DEG_TO_RAD(45-leftJoycon.gyro_y), DEG_TO_RAD(leftJoycon.gyro_z)); } else {	offset_orientation = HmdQuaternion_FromEulerAngles(DEG_TO_RAD(180-rightJoycon.gyro_x), DEG_TO_RAD(45-rightJoycon.gyro_y), DEG_TO_RAD(rightJoycon.gyro_z));}

	// Set the pose orientation to the hmd orientation with the offset applied.
	vr::HmdQuaternion_t reorient = HmdQuaternion_FromEulerAngles(0, DEG_TO_RAD(-90), 0);
	pose.qRotation = reorient * offset_orientation;

	// copy our position to our pose
	pose.vecPosition[ 0 ] = position.v[ 0 ];
	pose.vecPosition[ 1 ] = position.v[ 1 ];
	pose.vecPosition[ 2 ] = position.v[ 2 ];

	// The pose we provided is valid.
	// This should be set is
	pose.poseIsValid = IsHMDTracking();

	// Our device is always connected.
	// In reality with physical devices, when they get disconnected,
	// set this to false and icons in SteamVR will be updated to show the device is disconnected
	pose.deviceIsConnected = true;

	// The state of our tracking. For our virtual device, it's always going to be ok,
	// but this can get set differently to inform the runtime about the state of the device's tracking
	// and update the icons to inform the user accordingly.
	pose.result = vr::TrackingResult_Running_OK;
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated( role, pose, sizeof( vr::DriverPose_t ) );
	return;
}

void MyControllerDeviceDriver::MyPoseMountThread()
{
	while (true)
	{
		if (!leftJoycon.isConnected)
		{
			joycon_init_left();
		}
		if (!rightJoycon.isConnected)
		{
			joycon_init_right();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

vr::DriverPose_t pose;

void MyControllerDeviceDriver::MyPoseUpdateThread()
{	
	while ( is_active_ )
	{
		while (leftJoycon.isConnected || rightJoycon.isConnected)
		{
			if ((leftJoycon.isConnected) && (my_controller_role_ == vr::TrackedControllerRole_LeftHand)) {
				auto now = std::chrono::steady_clock::now();
				double dt = std::chrono::duration<double>(now - last_update_time_l).count();
				last_update_time_l = now;
				update_joycon_l(dt);
			}
			// if ((rightJoycon.isConnected) && (my_controller_role_ == vr::TrackedControllerRole_RightHand)) {
			// 	auto now = std::chrono::steady_clock::now();
			// 	double dt = std::chrono::duration<double>(now - last_update_time_r).count();
			// 	last_update_time_r = now;
			// 	update_joycon_r(dt);
			// }
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}		
	}
}


//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver when the device should enter standby mode.
// The device should be put into whatever low power mode it has.
// We don't really have anything to do here, so let's just log something.
//-----------------------------------------------------------------------------
void MyControllerDeviceDriver::EnterStandby()
{
	DriverLog( "%s hand has been put on standby", my_controller_role_ == vr::TrackedControllerRole_LeftHand ? "Left" : "Right" );
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver when the device should deactivate.
// This is typically at the end of a session
// The device should free any resources it has allocated here.
//-----------------------------------------------------------------------------
void MyControllerDeviceDriver::Deactivate()
{
	// Let's join our pose thread that's running
	// by first checking then setting is_active_ to false to break out
	// of the while loop, if it's running, then call .join() on the thread
	if ( is_active_.exchange( false ) )
	{
		joycon_deinit();
		my_pose_update_thread_.join();
	}

	// unassign our controller index (we don't want to be calling vrserver anymore after Deactivate() has been called
	my_controller_index_ = vr::k_unTrackedDeviceIndexInvalid;
}


//-----------------------------------------------------------------------------
// Purpose: This is called by our IServerTrackedDeviceProvider when its RunFrame() method gets called.
// It's not part of the ITrackedDeviceServerDriver interface, we created it ourselves.
//-----------------------------------------------------------------------------

void MyControllerDeviceDriver::MyRunFrame()
{
	if (rightJoycon.isConnected && my_controller_role_ == vr::TrackedControllerRole_RightHand){
		//get right inputs
		vr::VRDriverInput()->UpdateScalarComponent(input_handles_[MyComponent_stick_x], rightJoycon.stick_x, 0);
		vr::VRDriverInput()->UpdateScalarComponent(input_handles_[MyComponent_stick_y], rightJoycon.stick_y, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_stick_click], rightJoycon.stick_click, 0);
		
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_a_click], rightJoycon.a, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_b_click], rightJoycon.b, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_x_click], rightJoycon.x, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_y_click], rightJoycon.y, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_trigger_click], rightJoycon.shoulder, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_trigger_z_click], rightJoycon.z_shoulder, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_sl_click], rightJoycon.sl_bumper, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_sr_click], rightJoycon.sr_bumper, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_capture_click], rightJoycon.shutter_button, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_plus_minus_click], rightJoycon.pause_button, 0);

		update_joy_pose(my_controller_role_);
	} else if (leftJoycon.isConnected && my_controller_role_ == vr::TrackedControllerRole_LeftHand){
		//get left inputs
		
		vr::VRDriverInput()->UpdateScalarComponent(input_handles_[MyComponent_stick_x], leftJoycon.stick_x, 0);
		vr::VRDriverInput()->UpdateScalarComponent(input_handles_[MyComponent_stick_y], leftJoycon.stick_y, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_stick_click], leftJoycon.stick_click, 0);
		
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_a_click], leftJoycon.a, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_b_click], leftJoycon.b, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_x_click], leftJoycon.x, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_y_click], leftJoycon.y, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_trigger_click], leftJoycon.shoulder, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_trigger_z_click], leftJoycon.z_shoulder, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_sl_click], leftJoycon.sl_bumper, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_sr_click], leftJoycon.sr_bumper, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_capture_click], leftJoycon.shutter_button, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(input_handles_[MyComponent_plus_minus_click], leftJoycon.pause_button, 0);

		update_joy_pose(my_controller_role_);
	}
}


//-----------------------------------------------------------------------------
// Purpose: This is called by our IServerTrackedDeviceProvider when it pops an event off the event queue.
// It's not part of the ITrackedDeviceServerDriver interface, we created it ourselves.
//-----------------------------------------------------------------------------
void MyControllerDeviceDriver::MyProcessEvent( const vr::VREvent_t &vrevent )
{
	switch ( vrevent.eventType )
	{
		// Listen for haptic events
		case vr::VREvent_Input_HapticVibration:
		{
			// We now need to make sure that the event was intended for this device.
			// So let's compare handles of the event and our haptic component

			if ( vrevent.data.hapticVibration.componentHandle == input_handles_[ MyComponent_haptic ] )
			{
				// The event was intended for us!
				// To convert the data to a pulse, see the docs.
				// For this driver, we'll just print the values.

				float duration = vrevent.data.hapticVibration.fDurationSeconds;
				float frequency = vrevent.data.hapticVibration.fFrequency;
				float amplitude = vrevent.data.hapticVibration.fAmplitude;

				DriverLog( "Haptic event triggered for %s hand. Duration: %.2f, Frequency: %.2f, Amplitude: %.2f", my_controller_role_ == vr::TrackedControllerRole_LeftHand ? "left" : "right",
					duration, frequency, amplitude );
			}
			break;
		}
		default:
			break;
	}
}

//-----------------------------------------------------------------------------
// Purpose: Our IServerTrackedDeviceProvider needs our serial number to add us to vrserver.
// It's not part of the ITrackedDeviceServerDriver interface, we created it ourselves.
//-----------------------------------------------------------------------------
const std::string &MyControllerDeviceDriver::MyGetSerialNumber()
{
	return my_controller_serial_number_;
}