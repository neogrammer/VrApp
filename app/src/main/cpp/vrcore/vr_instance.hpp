//
// Created by jlhar on 12/5/2025.
//
#include <DebugOutput.h>
#include <GraphicsAPI_OpenGL_ES.h>
#include <OpenXRDebugUtils.h>

#ifndef VRAPP_VR_INSTANCE_HPP
#define VRAPP_VR_INSTANCE_HPP
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

class VrInstance {



    XrInstance vrInstance = {};
    std::vector<const char *> activeAPILayers = {};
    std::vector<const char *> activeExtensions = {};
    std::vector<std::string> apiLayers = {};
    std::vector<std::string> extensions = {};
    bool created{};
    bool hasPassthrough{};

public:
    // Function pointers
    PFN_xrCreatePassthroughFB                xrCreatePassthroughFB_ = nullptr;
    PFN_xrDestroyPassthroughFB               xrDestroyPassthroughFB_ = nullptr;
    PFN_xrPassthroughStartFB                 xrPassthroughStartFB_ = nullptr;
    PFN_xrPassthroughPauseFB                 xrPassthroughPauseFB_ = nullptr;
    PFN_xrCreatePassthroughLayerFB           xrCreatePassthroughLayerFB_ = nullptr;
    PFN_xrDestroyPassthroughLayerFB          xrDestroyPassthroughLayerFB_ = nullptr;
    PFN_xrPassthroughLayerPauseFB            xrPassthroughLayerPauseFB_ = nullptr;
    PFN_xrPassthroughLayerResumeFB           xrPassthroughLayerResumeFB_ = nullptr;
    PFN_xrPassthroughLayerSetStyleFB         xrPassthroughLayerSetStyleFB_ = nullptr;
    PFN_xrCreateGeometryInstanceFB           xrCreateGeometryInstanceFB_ = nullptr;
    PFN_xrDestroyGeometryInstanceFB          xrDestroyGeometryInstanceFB_ = nullptr;
    PFN_xrGeometryInstanceSetTransformFB     xrGeometryInstanceSetTransformFB_ = nullptr;


    void create();
    void destroy();
    XrInstance& get();

    bool isCreated();
    void getProperties();

    std::vector<const char *>& getActiveExtensions();
    std::vector<const char *>& getActiveLayers();
    void InitPassthrough(XrSession* session_, int flags_, XrPassthroughFB* passthrough_, XrPassthroughLayerFB* passthroughLayer_ );
    XrResult LoadPassthroughFunctions() {

//        if (!hasPassthrough)
//        {
//            XR_TUT_LOG_ERROR("We do not have the passthrough extension");
//            return XrResult::XR_ERROR_INITIALIZATION_FAILED;
//        }

        auto load = [&](const char* name, PFN_xrVoidFunction* fn) {
            XrResult r = xrGetInstanceProcAddr(vrInstance, name, fn);
            if (XR_FAILED(r)) {
                XR_TUT_LOG_ERROR("xrGetInstanceProcAddr failed");
            }
            return r;
        };

        XrResult overall = XR_SUCCESS;

        overall = (XrResult)(overall | load("xrCreatePassthroughFB",          (PFN_xrVoidFunction*)&xrCreatePassthroughFB_));
        overall = (XrResult)(overall | load("xrDestroyPassthroughFB",         (PFN_xrVoidFunction*)&xrDestroyPassthroughFB_));
        overall = (XrResult)(overall | load("xrPassthroughStartFB",           (PFN_xrVoidFunction*)&xrPassthroughStartFB_));
        overall = (XrResult)(overall | load("xrPassthroughPauseFB",           (PFN_xrVoidFunction*)&xrPassthroughPauseFB_));
        overall = (XrResult)(overall | load("xrCreatePassthroughLayerFB",     (PFN_xrVoidFunction*)&xrCreatePassthroughLayerFB_));
        overall = (XrResult)(overall | load("xrDestroyPassthroughLayerFB",    (PFN_xrVoidFunction*)&xrDestroyPassthroughLayerFB_));
        overall = (XrResult)(overall | load("xrPassthroughLayerPauseFB",      (PFN_xrVoidFunction*)&xrPassthroughLayerPauseFB_));
        overall = (XrResult)(overall | load("xrPassthroughLayerResumeFB",     (PFN_xrVoidFunction*)&xrPassthroughLayerResumeFB_));
        overall = (XrResult)(overall | load("xrPassthroughLayerSetStyleFB",   (PFN_xrVoidFunction*)&xrPassthroughLayerSetStyleFB_));
        overall = (XrResult)(overall | load("xrCreateGeometryInstanceFB",     (PFN_xrVoidFunction*)&xrCreateGeometryInstanceFB_));
        overall = (XrResult)(overall | load("xrDestroyGeometryInstanceFB",    (PFN_xrVoidFunction*)&xrDestroyGeometryInstanceFB_));
        overall = (XrResult)(overall | load("xrGeometryInstanceSetTransformFB",(PFN_xrVoidFunction*)&xrGeometryInstanceSetTransformFB_));

        return overall;
    }
};


#endif //VRAPP_VR_INSTANCE_HPP
