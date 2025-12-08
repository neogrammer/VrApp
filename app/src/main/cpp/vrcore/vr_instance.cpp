//
// Created by jlhar on 12/5/2025.
//

#include "vr_instance.hpp"
#include <OpenXRHelper.h>

// Function POINTERS for XR_FB_passthrough


void VrInstance::create() {
    XrApplicationInfo AI;
    strncpy(AI.applicationName, "VrApp", XR_MAX_APPLICATION_NAME_SIZE);
    AI.applicationVersion = 1;
    strncpy(AI.engineName, "VrApp", XR_MAX_ENGINE_NAME_SIZE);
    AI.engineVersion = 1;
    AI.apiVersion = XR_CURRENT_API_VERSION;
    extensions.push_back(XR_EXT_DEBUG_UTILS_EXTENSION_NAME);
    extensions.push_back(XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME);
    extensions.push_back(XR_EXT_HAND_TRACKING_EXTENSION_NAME);
    extensions.push_back(XR_EXT_HAND_INTERACTION_EXTENSION_NAME);
    extensions.push_back(XR_FB_PASSTHROUGH_EXTENSION_NAME);

    uint32_t apiLayerCount = 0;
    std::vector<XrApiLayerProperties> apiLayerProperties;
    OPENVR_CHECKi(xrEnumerateApiLayerProperties(0, &apiLayerCount, nullptr), "Failed to enumerate ApiLayerProperties.");
    apiLayerProperties.resize(apiLayerCount, {XR_TYPE_API_LAYER_PROPERTIES});
    OPENVR_CHECKi(xrEnumerateApiLayerProperties(apiLayerCount, &apiLayerCount, apiLayerProperties.data()), "Failed to enumerate ApiLayerProperties.");
    for (auto &requestLayer : apiLayers)
    {
        for (auto &layerProperty : apiLayerProperties)
        {
            // strcmp returns 0 if the strings match.
            if (strcmp(requestLayer.c_str(), layerProperty.layerName) != 0)
            {
                continue;
            }
            else
            {
                activeAPILayers.push_back(requestLayer.c_str());
                break;
            }
        }
    }
    uint32_t extensionCount = 0;
    std::vector<XrExtensionProperties> extensionProperties;
    OPENVR_CHECKi(xrEnumerateInstanceExtensionProperties(nullptr, 0, &extensionCount, nullptr), "Failed to enumerate InstanceExtensionProperties.");
    extensionProperties.resize(extensionCount);
    for (auto &e : extensionProperties) {
        e.type = XR_TYPE_EXTENSION_PROPERTIES;
        e.next = nullptr;
    }
    OPENVR_CHECKi(xrEnumerateInstanceExtensionProperties(nullptr, extensionCount, &extensionCount, extensionProperties.data()), "Failed to enumerate InstanceExtensionProperties.");

    bool hasPassthrough = false;
    for (auto& e : extensionProperties) {
        if (strcmp(e.extensionName, XR_FB_PASSTHROUGH_EXTENSION_NAME) == 0) {
            hasPassthrough = true;
            break;
        }
    }

    XR_TUT_LOG("XR_FB_passthrough is supported");


    for (auto &requestedInstanceExtension : extensions)
    {
        bool found = false;
        for (auto &extensionProperty : extensionProperties)
        {
            // strcmp returns 0 if the strings match.
            if (strcmp(requestedInstanceExtension.c_str(), extensionProperty.extensionName) != 0)
            {
                continue;
            }
            else
            {
                activeExtensions.push_back(requestedInstanceExtension.c_str());
                found = true;
                break;
            }
        }
        if (!found)
        {
            XR_TUT_LOG_ERROR("Failed to find OpenXR instance extension: " << requestedInstanceExtension);
        }
    }
    XrInstanceCreateInfo instanceCI{XR_TYPE_INSTANCE_CREATE_INFO};
    instanceCI.createFlags = 0;
    instanceCI.applicationInfo = AI;
    instanceCI.enabledApiLayerCount = static_cast<uint32_t>(activeAPILayers.size());
    instanceCI.enabledApiLayerNames = activeAPILayers.data();
    instanceCI.enabledExtensionCount = static_cast<uint32_t>(activeExtensions.size());
    instanceCI.enabledExtensionNames = activeExtensions.data();


    OPENVR_CHECKi(xrCreateInstance(&instanceCI, &vrInstance), "Failed to create Instance.");
    created = true;
}


XrInstance &VrInstance::get()
{
    return vrInstance;
}

bool VrInstance::isCreated()
{
    return created;
}

void VrInstance::destroy()
{
    OPENVR_CHECKi(xrDestroyInstance(vrInstance), "Failed to destroy Instance.");
}

void VrInstance::getProperties()
{
    XrInstanceProperties instanceProperties{XR_TYPE_INSTANCE_PROPERTIES};
    OPENVR_CHECKi(xrGetInstanceProperties(vrInstance, &instanceProperties), "Failed to get InstanceProperties.");

    XR_TUT_LOG("OpenXR Runtime: " << instanceProperties.runtimeName << " - "
                                  << XR_VERSION_MAJOR(instanceProperties.runtimeVersion) << "."
                                  << XR_VERSION_MINOR(instanceProperties.runtimeVersion) << "."
                                  << XR_VERSION_PATCH(instanceProperties.runtimeVersion));
}

std::vector<const char *> & VrInstance::getActiveExtensions() {
    return activeExtensions;
}

std::vector<const char *> &VrInstance::getActiveLayers() {
    return activeAPILayers;
}

void VrInstance::InitPassthrough(XrSession* session_, int flags_, XrPassthroughFB* passthrough_, XrPassthroughLayerFB* passthroughLayer_ )
{
    // 1) Create passthrough object
    XrPassthroughCreateInfoFB ptInfo{XR_TYPE_PASSTHROUGH_CREATE_INFO_FB};
    ptInfo.flags = flags_;
    OPENVR_CHECKi(xrCreatePassthroughFB_(*session_, &ptInfo, passthrough_),"Unable to create the passthrough");

    XrPassthroughLayerCreateInfoFB layerInfo{XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB};
    layerInfo.passthrough = *passthrough_;
    layerInfo.purpose     = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;
    layerInfo.flags       = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
    OPENVR_CHECKi(xrCreatePassthroughLayerFB_(*session_, &layerInfo, passthroughLayer_), "cant create passthrough layer");

    XR_TUT_LOG("Passthrough initialized OK");
}