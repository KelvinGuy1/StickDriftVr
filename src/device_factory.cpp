#include "device_provider.h"
#include <cstring>
#include "openvr_driver.h"

DeviceProvider deviceProvider;

#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))

HMD_DLL_EXPORT void* HmdDriverFactory(const char* interfaceName, int* returnCode){
    if (strcmp(interfaceName, vr::IServerTrackedDeviceProvider_Version) == 0)
    {
        return &deviceProvider;
    }

    if (returnCode)
    {
        *returnCode = vr::VRInitError_Init_InterfaceNotFound;
    }

    return NULL;    
}
