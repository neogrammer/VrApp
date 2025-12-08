#include "audio_engine.h"
#include "vrcore/vr_instance.hpp"
// include xr linear algebra for XrVector and XrMatrix classes.
#include <xr_linear_algebra.h>
// Declare some useful operators for vectors:
XrVector3f operator-(XrVector3f a, XrVector3f b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
XrVector3f operator*(XrVector3f a, float b) {
    return {a.x * b, a.y * b, a.z * b};
}

XrVector3f operator+(XrVector3f a, XrVector3f b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

float Dot(XrVector3f a, XrVector3f b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

float Length(XrVector3f v) {
    return sqrtf(Dot(v, v));
}

XrVector3f Normalize(XrVector3f v) {
    float len = Length(v);
    if (len <= 1e-6f) return {0,0,0};
    float inv = 1.0f / len;
    return {v.x * inv, v.y * inv, v.z * inv};
}

// rotate vector v by quaternion q (q * v * q^-1)
XrVector3f Rotate(const XrQuaternionf& q, XrVector3f v) {
    XrVector3f qv{q.x, q.y, q.z};
    float s = q.w;

    XrVector3f cross{
            qv.y * v.z - qv.z * v.y,
            qv.z * v.x - qv.x * v.z,
            qv.x * v.y - qv.y * v.x
    };

    XrVector3f term1 = v * (s*s - Dot(qv, qv));
    XrVector3f term2 = qv * (2.0f * Dot(qv, v));
    XrVector3f term3 = cross * (2.0f * s);

    return term1 + term2 + term3;
}

XrQuaternionf LookRotation(XrVector3f dir, XrVector3f up) {
    dir = Normalize(dir);
    up  = Normalize(up);

    XrVector3f y = dir;
    XrVector3f x{
            up.y * y.z - up.z * y.y,
            up.z * y.x - up.x * y.z,
            up.x * y.y - up.y * y.x
    };
    x = Normalize(x);
    XrVector3f z{
            x.y * y.z - x.z * y.y,
            x.z * y.x - x.x * y.z,
            x.x * y.y - x.y * y.x
    };

    float m00=x.x, m01=y.x, m02=z.x;
    float m10=x.y, m11=y.y, m12=z.y;
    float m20=x.z, m21=y.z, m22=z.z;

    float tr = m00 + m11 + m22;
    XrQuaternionf q{};
    if (tr > 0.0f) {
        float S = sqrtf(tr + 1.0f) * 2.0f;
        q.w = 0.25f * S;
        q.x = (m21 - m12) / S;
        q.y = (m02 - m20) / S;
        q.z = (m10 - m01) / S;
    } else {
        q = {0,0,0,1};
    }
    return q;
}

XrQuaternionf Mul(const XrQuaternionf& a, const XrQuaternionf& b)
{
    XrQuaternionf r;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    return r;
}

// Include <algorithm> for std::min and max
#include <algorithm>
// A deque is used to track the blocks to draw.
#include <deque>
// Random numbers for colorful blocks
#include <random>
static std::uniform_real_distribution<float> pseudorandom_distribution(0, 1.f);
static std::mt19937 pseudo_random_generator;





PFN_xrCreateHandTrackerEXT xrCreateHandTrackerEXT = nullptr;
PFN_xrDestroyHandTrackerEXT xrDestroyHandTrackerEXT = nullptr;
PFN_xrLocateHandJointsEXT xrLocateHandJointsEXT = nullptr;




class VrApp {

    AudioEngine m_audioEngine;

    VrInstance vrInstance = {};
    XrDebugUtilsMessengerEXT m_debugUtilsMessenger = {};
    XrFormFactor m_formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrSystemId m_systemID = {};
    XrSystemProperties m_systemProperties = {XR_TYPE_SYSTEM_PROPERTIES};
    GraphicsAPI_Type m_apiType = UNKNOWN;
    XrSessionState m_sessionState = XR_SESSION_STATE_UNKNOWN;

    XrTime m_lastPredictedDisplayTime = 0;

    XrPassthroughFB      m_passthrough      = XR_NULL_HANDLE;
    XrPassthroughLayerFB m_passthroughLayer = XR_NULL_HANDLE;

    bool m_applicationRunning = true;
    bool m_sessionRunning = false;

    std::vector<XrViewConfigurationType> m_applicationViewConfigurations = {XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO};
    std::vector<XrViewConfigurationType> m_viewConfigurations;
    XrViewConfigurationType m_viewConfiguration = XR_VIEW_CONFIGURATION_TYPE_MAX_ENUM;
    std::vector<XrViewConfigurationView> m_viewConfigurationViews;

    struct SwapchainInfo {
        XrSwapchain swapchain = XR_NULL_HANDLE;
        int64_t swapchainFormat = 0;
        std::vector<void *> imageViews;
    };
    std::vector<SwapchainInfo> m_colorSwapchainInfos = {};
    std::vector<SwapchainInfo> m_depthSwapchainInfos = {};

    std::vector<XrEnvironmentBlendMode> m_applicationEnvironmentBlendModes = { XR_ENVIRONMENT_BLEND_MODE_OPAQUE, XR_ENVIRONMENT_BLEND_MODE_ADDITIVE, XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND   };
    std::vector<XrEnvironmentBlendMode> m_environmentBlendModes = {};
    XrEnvironmentBlendMode m_environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_MAX_ENUM;

    XrSpace m_localSpace = XR_NULL_HANDLE;
    struct RenderLayerInfo {
        XrTime predictedDisplayTime;
        std::vector<XrCompositionLayerBaseHeader *> layers;
        XrCompositionLayerProjection layerProjection = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        std::vector<XrCompositionLayerProjectionView> layerProjectionViews;
        std::vector<XrCompositionLayerDepthInfoKHR> layerDepthInfos;
        XrCompositionLayerPassthroughFB passthroughLayer = {XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB};
    };

    float m_viewHeightM = 1.5f;

    void *m_vertexBuffer = nullptr;
    void *m_indexBuffer = nullptr;
    void *m_uniformBuffer_Camera = nullptr;
    void *m_uniformBuffer_Normals = nullptr;
    void *m_vertexShader = nullptr, *m_fragmentShader = nullptr;
    void *m_pipeline = nullptr;

    // An instance of a 3d colored block.
    struct Block {
        XrPosef pose;
        XrVector3f scale;
        XrVector3f color;
    };

    static constexpr int MAX_DRUMS = 8;

    struct Drum {
        XrVector3f center;   // world-space in LOCAL space frame
        XrVector3f normal;   // usually (0,1,0)
        float      radius;   // trigger radius for hit detection
        int        soundId;  // used later when you play audio
    };

    struct Stick {
        XrPosef    pose;              // pose in world
        int        heldBy;            // -1 = free, 0 = left hand, 1 = right hand
        float      length;            // meters (model-dependent)
        XrVector3f tipLocal;          // tip position in local coords
        float      lastHeight[MAX_DRUMS]; // previous frame Y-height over each drum
        bool       active;            // stick currently available

        // Bounce state
        float      bounceTime;        // seconds remaining in bounce
        XrVector3f bounceDir;         // direction of bounce (drum normal)
        XrQuaternionf localRot;
    };

    Drum  m_drums[MAX_DRUMS];
    int   m_drumCount = 0;

    Stick m_sticks[2];

// The list of block instances.
    std::deque<Block> m_blocks;
// Don't let too many m_blocks get created.
    const size_t m_maxBlockCount = 100;
// Which block, if any, is being held by each of the user's hands or controllers.
    int m_grabbedBlock[2] = {-1, -1};
// Which block, if any, is nearby to each hand or controller.
    int m_nearBlock[2] = {-1, -1};

    XrActionSet m_actionSet;
// An action for grabbing blocks, and an action to change the color of a block.
    XrAction m_grabCubeAction, m_spawnCubeAction, m_changeColorAction;
// The realtime states of these actions.
    XrActionStateFloat m_grabState[2] = {{XR_TYPE_ACTION_STATE_FLOAT}, {XR_TYPE_ACTION_STATE_FLOAT}};
    XrActionStateBoolean m_changeColorState[2] = {{XR_TYPE_ACTION_STATE_BOOLEAN}, {XR_TYPE_ACTION_STATE_BOOLEAN}};
    XrActionStateBoolean m_spawnCubeState = {XR_TYPE_ACTION_STATE_BOOLEAN};
// The haptic output action for grabbing cubes.
    XrAction m_buzzAction;
// The current haptic output value for each controller.
    float m_buzz[2] = {0, 0};
// The action for getting the hand or controller position and orientation.
    XrAction m_palmPoseAction;
// The XrPaths for left and right hand hands or controllers.
    XrPath m_handPaths[2] = {0, 0};
// The spaces that represents the two hand poses.
    XrSpace m_handPoseSpace[2];
    XrActionStatePose m_handPoseState[2] = {{XR_TYPE_ACTION_STATE_POSE}, {XR_TYPE_ACTION_STATE_POSE}};
// The current poses obtained from the XrSpaces.
    XrPosef m_handPose[2] = {
            {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -m_viewHeightM}},
            {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -m_viewHeightM}}};


    // The hand tracking properties, namely, is it supported?
    XrSystemHandTrackingPropertiesEXT handTrackingSystemProperties = {XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
// Each tracked hand has a live list of joint locations.
    struct Hand {
        XrHandJointLocationEXT m_jointLocations[XR_HAND_JOINT_COUNT_EXT];
        XrHandTrackerEXT m_handTracker = 0;
    };
    Hand m_hands[2];




public:
    static android_app *androidApp;
    struct AndroidAppState
    {
        ANativeWindow *nativeWindow = nullptr;
        bool resumed = false;
    };
    static AndroidAppState androidAppState;
    static void AndroidAppHandleCmd(struct android_app *app, int32_t cmd);
    std::unique_ptr<GraphicsAPI> m_graphicsAPI = nullptr;
    XrSession m_session = XR_NULL_HANDLE;

public:
    explicit VrApp(GraphicsAPI_Type apiType)
        : m_apiType{apiType}
    {
        if (!CheckGraphicsAPI_TypeIsValidForPlatform(m_apiType))
        {
            std::cout << "ERROR: The provided Graphics API is not valid for this platform." << std::endl;
            DEBUG_BREAK;
        }
    }
    ~VrApp() = default;
    void Run()
    {
        vrInstance.create();


            OPENVR_CHECK(xrGetInstanceProcAddr(vrInstance.get(), "xrCreateHandTrackerEXT", (PFN_xrVoidFunction *)&xrCreateHandTrackerEXT), "Failed to get xrCreateHandTrackerEXT.");
            OPENVR_CHECK(xrGetInstanceProcAddr(vrInstance.get(), "xrDestroyHandTrackerEXT", (PFN_xrVoidFunction *)&xrDestroyHandTrackerEXT), "Failed to get xrDestroyHandTrackerEXT.");
            OPENVR_CHECK(xrGetInstanceProcAddr(vrInstance.get(), "xrLocateHandJointsEXT", (PFN_xrVoidFunction *)&xrLocateHandJointsEXT), "Failed to get xrLocateHandJointsEXT.");

            vrInstance.LoadPassthroughFunctions();

            CreateDebugMessenger();

            vrInstance.getProperties();

        GetSystemID();
        CreateActionSet();
        SuggestBindings();
        GetViewConfigurationViews();
        GetEnvironmentBlendModes();

        CreateSession();
        vrInstance.InitPassthrough(&m_session, XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB, &m_passthrough, &m_passthroughLayer);
        CreateActionPoses();
        AttachActionSet();
        if (handTrackingSystemProperties.supportsHandTracking) {
            CreateHandTrackers();
        }
        CreateReferenceSpace();
        CreateSwapchains();
        CreateResources();
        while (m_applicationRunning) {
            PollSystemEvents();
            PollEvents();
            if (m_sessionRunning) {
                RenderFrame();
            }
        }

        DestroyResources();
        DestroySwapchains();
        DestroyReferenceSpace();
        DestroySession();
        if (vrInstance.isCreated())
        {
            DestroyDebugMessenger();
            vrInstance.destroy();
        }
    }
private:


    void InitDrumsAndSticks()
    {
        m_drumCount = 0;

        auto addDrum = [&](XrVector3f center, float radius, int soundId) {
            if (m_drumCount >= MAX_DRUMS) return;
            Drum& d = m_drums[m_drumCount++];
            d.center = center;
            d.normal = {0.0f, 1.0f, 0.0f}; // flat, facing up
            d.radius = radius;
            d.soundId = soundId;
        };

        // ONE HUGE TEST DRUM:
        // Dead center in front of the head at z = -1
        addDrum({0.0f, -0.3f, -0.55f}, 0.35f, 0);

        // TWO STICKS: left/right of center, also in front of the head
        for (int i = 0; i < 2; ++i) {
            Stick& s = m_sticks[i];

            s.heldBy = -1;
            s.length = 0.35f;

// In *stick space*, the tip is along +Y (long axis of the cuboid)
            s.tipLocal = {0.0f, s.length * 0.75f, 0.0f};

            s.pose.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
            s.pose.position = {
                    (i == 0 ? -0.15f : 0.15f),
                    -0.3f,
                    -0.3f
            };

            for (int d = 0; d < MAX_DRUMS; ++d) {
                s.lastHeight[d] = -0.3f;
            }
            s.active     = true;
            s.bounceTime = 0.0f;
            s.bounceDir  = {0.0f, 0.0f, 0.0f};

// NEW: rotate the stick -90° around X so +Y (stick) points more "forward"
// instead of "up the middle finger".
            {
                const float angle = -3.14159265f * 0.5f;  // -90 degrees in radians
                const float half  = 0.5f * angle;
                const float sAng  = sinf(half);
                const float cAng  = cosf(half);

                s.localRot = { sAng, 0.0f, 0.0f, cAng }; // axis (1,0,0), angle -90°
            }
        }
    }

    XrVector3f StickTipWorld(const Stick& s) const
    {
        // world position of the tip from grip + local offset
        return s.pose.position + Rotate(s.pose.orientation, s.tipLocal);
    }

    void AutoDropStick(Stick& s)
    {
        s.heldBy = -1;
        s.pose.position   = { 0.25f, 0.0f, 0.3f };  // resting position near player
        s.pose.orientation = {0,0,0,1};
        for (int d = 0; d < MAX_DRUMS; ++d) {
            s.lastHeight[d] = 0.0f;
        }

        s.bounceTime = 0.0f;
        s.bounceDir  = {0.0f, 0.0f, 0.0f};
    }

    void UpdateSticks(float dt)
    {
        const float grabRadius = 0.15f;

        for (int i = 0; i < 2; ++i) {
            Stick& s = m_sticks[i];

            // If stick is held, follow that hand pose
            if (s.heldBy == 0 || s.heldBy == 1) {
                int hand = s.heldBy;
                if (m_handPoseState[hand].isActive) {
                    XrPosef handPose = m_handPose[hand];

                    s.pose.position    = handPose.position;
                    s.pose.orientation = Mul(handPose.orientation, s.localRot);
                } else {
                    // Hand lost tracking -> drop stick
                    AutoDropStick(s);
                }
            } else {
                // Not held: see if a hand is close + squeezed
                for (int hand = 0; hand < 2; ++hand) {
                    if (!m_handPoseState[hand].isActive) continue;
                    if (!m_grabState[hand].isActive ||
                        m_grabState[hand].currentState < 0.5f) continue;

                    XrVector3f diff = m_handPose[hand].position - s.pose.position;
                    float dist = Length(diff);
                    if (dist < grabRadius) {
                        s.heldBy = hand;
                        break;
                    }
                }
            }
            // Apply bounce AFTER you’ve aligned the stick to the hand pose
            if (s.bounceDir.x != 0.0f || s.bounceDir.y != 0.0f || s.bounceDir.z != 0.0f) {
                const float totalBounce = 0.30f;   // 0.3s = more obvious
                const float maxAmp      = 0.03f;   // 3cm max offset

                // advance time
                s.bounceTime += dt;

                float tNorm = s.bounceTime / totalBounce;
                if (tNorm >= 1.0f) {
                    // bounce finished
                    s.bounceTime = 0.0f;
                    s.bounceDir  = {0.0f, 0.0f, 0.0f};
                } else {
                    // one-lobe sine: 0 -> 0 -> 0 with peak in the middle
                    float offset = sinf(tNorm * 3.14159265f) * maxAmp;
                    s.pose.position = s.pose.position + s.bounceDir * offset;
                }
            }
        }
    }

    bool TestStickHitDrum(Stick& s, int drumIndex, float dt)
    {
        if (!s.active) return false;
        if (drumIndex < 0 || drumIndex >= m_drumCount) return false;

        Drum& d = m_drums[drumIndex];

        XrVector3f tip = StickTipWorld(s);

        // Use a plane at the *top* of the visual drum, not the center
        const float headOffset = 0.03f * 0.5f; // must match your visual drum half-thickness
        XrVector3f planePoint  = d.center + d.normal * headOffset;

        XrVector3f toTip  = tip - planePoint;

        // height above drum head plane
        float height = Dot(toTip, d.normal);

        // radial distance in the plane
        XrVector3f radial = toTip - d.normal * height;
        float      r      = Length(radial);

        float lastH = s.lastHeight[drumIndex];

        float collisionRadius = d.radius * 0.5f;
        bool inside   = (r <= collisionRadius);
        bool crossing = (height <= 0.0f && lastH > 0.0f); // crossed through head plane

        if (inside && crossing) {
            float v = (lastH - height) / std::max(dt, 0.0001f); // downward speed

            if (v > 0.5f) {
                // Play snare for now (soundId could choose different samples later)
                if (d.soundId == 0) {
                    m_audioEngine.playSnare();
                }

                // Start bounce animation
                s.bounceTime = 0.0f;
                s.bounceDir  = d.normal;
            }
        }

        // Simple clamp so the tip doesn't ghost through the head
        if (inside && height < 0.0f) {
            float penetration = height; // negative

            // Move the whole stick up along the normal
            s.pose.position = s.pose.position - d.normal * penetration;
        }

        s.lastHeight[drumIndex] = height;
        return inside && crossing;
    }

    void UpdateDrumHits(float dt)
    {
        for (int sIdx = 0; sIdx < 2; ++sIdx) {
            Stick& s = m_sticks[sIdx];
            if (!s.active) continue;

            for (int d = 0; d < m_drumCount; ++d) {
                TestStickHitDrum(s, d, dt);
            }
        }
    }

    void CreateDebugMessenger()
    {
        // Check that "XR_EXT_debug_utils" is in the active Instance Extensions before creating an XrDebugUtilsMessengerEXT.
        if (IsStringInVector(vrInstance.getActiveExtensions(), XR_EXT_DEBUG_UTILS_EXTENSION_NAME))
        {
            m_debugUtilsMessenger = CreateOpenXRDebugUtilsMessenger(vrInstance.get());  // From OpenXRDebugUtils.h.
        }
    }
    void DestroyDebugMessenger()
    {
        // Check that "XR_EXT_debug_utils" is in the active Instance Extensions before destroying the XrDebugUtilsMessengerEXT.
        if (m_debugUtilsMessenger != XR_NULL_HANDLE)
        {
            DestroyOpenXRDebugUtilsMessenger(vrInstance.get(), m_debugUtilsMessenger);  // From OpenXRDebugUtils.h.
        }
    }

    void CreateSession()
    {
        XrSessionCreateInfo sessionCI{XR_TYPE_SESSION_CREATE_INFO};
        m_graphicsAPI = std::make_unique<GraphicsAPI_OpenGL_ES>(vrInstance.get(), m_systemID);
        sessionCI.next = m_graphicsAPI->GetGraphicsBinding();
        sessionCI.createFlags = 0;
        sessionCI.systemId = m_systemID;

        OPENVR_CHECK(xrCreateSession(vrInstance.get(), &sessionCI, &m_session), "Failed to create Session.");
    }
    void DestroySession()
    {
        // Destroy hand trackers (if you created them)
        if (xrDestroyHandTrackerEXT) {
            for (int i = 0; i < 2; ++i) {
                if (m_hands[i].m_handTracker != XR_NULL_HANDLE) {
                    xrDestroyHandTrackerEXT(m_hands[i].m_handTracker);
                    m_hands[i].m_handTracker = XR_NULL_HANDLE;
                }
            }
        }

        // Destroy passthrough layer (if any)
        if (m_passthroughLayer != XR_NULL_HANDLE && vrInstance.xrDestroyPassthroughLayerFB_) {
            vrInstance.xrDestroyPassthroughLayerFB_(m_passthroughLayer);
            m_passthroughLayer = XR_NULL_HANDLE;
        }

        // Destroy passthrough feature
        if (m_passthrough != XR_NULL_HANDLE && vrInstance.xrDestroyPassthroughFB_) {
            vrInstance.xrDestroyPassthroughFB_(m_passthrough);
            m_passthrough = XR_NULL_HANDLE;
        }

        // Finally destroy the session itself
        if (m_session != XR_NULL_HANDLE) {
            OPENVR_CHECK(xrDestroySession(m_session), "Failed to destroy Session.");
            m_session = XR_NULL_HANDLE;
        }
    }

    void GetSystemID()
    {
        XrSystemGetInfo systemGI{XR_TYPE_SYSTEM_GET_INFO};
        systemGI.formFactor = m_formFactor;
        OPENVR_CHECK(xrGetSystem(vrInstance.get(), &systemGI, &m_systemID), "Failed to get SystemID.");
        // Check if hand tracking is supported.
        m_systemProperties.next = &handTrackingSystemProperties;
        OPENVR_CHECK(xrGetSystemProperties(vrInstance.get(), m_systemID, &m_systemProperties), "Failed to get SystemProperties.");
    }

    XrPath CreateXrPath(const char *path_string) {
        XrPath xrPath;
        OPENVR_CHECK(xrStringToPath(vrInstance.get(), path_string, &xrPath), "Failed to create XrPath from string.");
        return xrPath;
    }
    std::string FromXrPath(XrPath path) {
        uint32_t strl;
        char text[XR_MAX_PATH_LENGTH];
        XrResult res;
        res = xrPathToString(vrInstance.get(), path, XR_MAX_PATH_LENGTH, &strl, text);
        std::string str;
        if (res == XR_SUCCESS) {
            str = text;
        } else {
            OPENVR_CHECK(res, "Failed to retrieve path.");
        }
        return str;
    }


    void CreateActionSet() {
        XrActionSetCreateInfo actionSetCI{XR_TYPE_ACTION_SET_CREATE_INFO};
        // The internal name the runtime uses for this Action Set.
        strncpy(actionSetCI.actionSetName, "vrapp-actionset",
                XR_MAX_ACTION_SET_NAME_SIZE);
        // Localized names are required so there is a human-readable action name to show the user if they are rebinding Actions in an options screen.
        strncpy(actionSetCI.localizedActionSetName, "VrApp ActionSet",
                XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE);
        // Set a priority: this comes into play when we have multiple Action Sets, and determines which Action takes priority in binding to a specific input.
        actionSetCI.priority = 0;

        OPENVR_CHECK(xrCreateActionSet(vrInstance.get(), &actionSetCI, &m_actionSet),
                     "Failed to create ActionSet.");

        auto CreateAction = [this](XrAction &xrAction, const char *name, XrActionType xrActionType, std::vector<const char *> subaction_paths = {}) -> void {
            XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
            // The type of action: float input, pose, haptic output etc.
            actionCI.actionType = xrActionType;
            // Subaction paths, e.g. left and right hand. To distinguish the same action performed on different devices.
            std::vector<XrPath> subaction_xrpaths;
            for (auto p : subaction_paths) {
                subaction_xrpaths.push_back(CreateXrPath(p));
            }
            actionCI.countSubactionPaths = (uint32_t)subaction_xrpaths.size();
            actionCI.subactionPaths = subaction_xrpaths.data();
            // The internal name the runtime uses for this Action.
            strncpy(actionCI.actionName, name, XR_MAX_ACTION_NAME_SIZE);
            // Localized names are required so there is a human-readable action name to show the user if they are rebinding the Action in an options screen.
            strncpy(actionCI.localizedActionName, name, XR_MAX_LOCALIZED_ACTION_NAME_SIZE);
            OPENVR_CHECK(xrCreateAction(m_actionSet, &actionCI, &xrAction), "Failed to create Action.");
        };

        // An Action for grabbing cubes.
        CreateAction(m_grabCubeAction, "grab-cube", XR_ACTION_TYPE_FLOAT_INPUT, {"/user/hand/left", "/user/hand/right"});
        CreateAction(m_spawnCubeAction, "spawn-cube", XR_ACTION_TYPE_BOOLEAN_INPUT);
        CreateAction(m_changeColorAction, "change-color", XR_ACTION_TYPE_BOOLEAN_INPUT, {"/user/hand/left", "/user/hand/right"});
        // An Action for the position of the palm of the user's hand - appropriate for the location of a grabbing Actions.
        CreateAction(m_palmPoseAction, "palm-pose", XR_ACTION_TYPE_POSE_INPUT, {"/user/hand/left", "/user/hand/right"});
        // An Action for a vibration output on one or other hand.
        CreateAction(m_buzzAction, "buzz", XR_ACTION_TYPE_VIBRATION_OUTPUT, {"/user/hand/left", "/user/hand/right"});
        // For later convenience we create the XrPaths for the subaction path names.
        m_handPaths[0] = CreateXrPath("/user/hand/left");
        m_handPaths[1] = CreateXrPath("/user/hand/right");

    }

    void SuggestBindings() {
        auto SuggestBindings = [this](const char *profile_path,
                                      std::vector<XrActionSuggestedBinding> bindings) -> bool {
            // The application can call xrSuggestInteractionProfileBindings once per interaction profile that it supports.
            XrInteractionProfileSuggestedBinding interactionProfileSuggestedBinding{
                    XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
            interactionProfileSuggestedBinding.interactionProfile = CreateXrPath(profile_path);
            interactionProfileSuggestedBinding.suggestedBindings = bindings.data();
            interactionProfileSuggestedBinding.countSuggestedBindings = (uint32_t) bindings.size();
            if (xrSuggestInteractionProfileBindings(vrInstance.get(),
                                                    &interactionProfileSuggestedBinding) ==
                XrResult::XR_SUCCESS)
                return true;
            XR_TUT_LOG("Failed to suggest bindings with " << profile_path);
            return false;
        };

        bool any_ok = false;
        // Each Action here has two paths, one for each SubAction path.
        any_ok |= SuggestBindings("/interaction_profiles/khr/simple_controller", {{m_changeColorAction, CreateXrPath("/user/hand/left/input/select/click")},
                                                                                  {m_grabCubeAction, CreateXrPath("/user/hand/right/input/select/click")},
                                                                                  {m_spawnCubeAction, CreateXrPath("/user/hand/right/input/menu/click")},
                                                                                  {m_palmPoseAction, CreateXrPath("/user/hand/left/input/grip/pose")},
                                                                                  {m_palmPoseAction, CreateXrPath("/user/hand/right/input/grip/pose")},
                                                                                  {m_buzzAction, CreateXrPath("/user/hand/left/output/haptic")},
                                                                                  {m_buzzAction, CreateXrPath("/user/hand/right/output/haptic")}});

        // Each Action here has two paths, one for each SubAction path.
        any_ok |= SuggestBindings("/interaction_profiles/oculus/touch_controller", {{m_grabCubeAction, CreateXrPath("/user/hand/left/input/squeeze/value")},
                                                                                    {m_grabCubeAction, CreateXrPath("/user/hand/right/input/squeeze/value")},
                                                                                    {m_spawnCubeAction, CreateXrPath("/user/hand/right/input/a/click")},
                                                                                    {m_changeColorAction, CreateXrPath("/user/hand/left/input/trigger/value")},
                                                                                    {m_changeColorAction, CreateXrPath("/user/hand/right/input/trigger/value")},
                                                                                    {m_palmPoseAction, CreateXrPath("/user/hand/left/input/grip/pose")},
                                                                                    {m_palmPoseAction, CreateXrPath("/user/hand/right/input/grip/pose")},
                                                                                    {m_buzzAction, CreateXrPath("/user/hand/left/output/haptic")},
                                                                                    {m_buzzAction, CreateXrPath("/user/hand/right/output/haptic")}});

        if (!any_ok) {
            DEBUG_BREAK;
        }
    }

    void CreateActionPoses() {
        // Create an xrSpace for a pose action.
        auto CreateActionPoseSpace = [this](XrSession session, XrAction xrAction, const char *subaction_path = nullptr) -> XrSpace {
            XrSpace xrSpace;
            const XrPosef xrPoseIdentity = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
            // Create frame of reference for a pose action
            XrActionSpaceCreateInfo actionSpaceCI{XR_TYPE_ACTION_SPACE_CREATE_INFO};
            actionSpaceCI.action = xrAction;
            actionSpaceCI.poseInActionSpace = xrPoseIdentity;
            if (subaction_path)
                actionSpaceCI.subactionPath = CreateXrPath(subaction_path);
            OPENVR_CHECK(xrCreateActionSpace(session, &actionSpaceCI, &xrSpace), "Failed to create ActionSpace.");
            return xrSpace;
        };
        m_handPoseSpace[0] = CreateActionPoseSpace(m_session, m_palmPoseAction, "/user/hand/left");
        m_handPoseSpace[1] = CreateActionPoseSpace(m_session, m_palmPoseAction, "/user/hand/right");
    }

    void AttachActionSet() {
        // Attach the action set we just made to the session. We could attach multiple action sets!
        XrSessionActionSetsAttachInfo actionSetAttachInfo{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
        actionSetAttachInfo.countActionSets = 1;
        actionSetAttachInfo.actionSets = &m_actionSet;
        OPENVR_CHECK(xrAttachSessionActionSets(m_session, &actionSetAttachInfo), "Failed to attach ActionSet to Session.");
    }

    void CreateHandTrackers() {
        for (int i = 0; i < 2; i++) {
            Hand &hand = m_hands[i];
            XrHandTrackerCreateInfoEXT xrHandTrackerCreateInfo = {XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
            xrHandTrackerCreateInfo.hand = i == 0 ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT;
            xrHandTrackerCreateInfo.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
            OPENVR_CHECK(xrCreateHandTrackerEXT(m_session, &xrHandTrackerCreateInfo, &hand.m_handTracker), "Failed to create Hand Tracker.");
        }
    }

    void RecordCurrentBindings() {
        if (m_session) {
            // now we are ready to:
            XrInteractionProfileState interactionProfile = {XR_TYPE_INTERACTION_PROFILE_STATE, 0, 0};
            // for each action, what is the binding?
            OPENVR_CHECK(xrGetCurrentInteractionProfile(m_session, m_handPaths[0], &interactionProfile), "Failed to get profile.");
            if (interactionProfile.interactionProfile) {
                XR_TUT_LOG("user/hand/left ActiveProfile " << FromXrPath(interactionProfile.interactionProfile).c_str());
            }
            OPENVR_CHECK(xrGetCurrentInteractionProfile(m_session, m_handPaths[1], &interactionProfile), "Failed to get profile.");
            if (interactionProfile.interactionProfile) {
                XR_TUT_LOG("user/hand/right ActiveProfile " << FromXrPath(interactionProfile.interactionProfile).c_str());
            }
        }
    }

    void GetViewConfigurationViews()
    {
        // Gets the View Configuration Types. The first call gets the count of the array that will be returned. The next call fills out the array.
        uint32_t viewConfigurationCount = 0;
        OPENVR_CHECK(xrEnumerateViewConfigurations(vrInstance.get(), m_systemID, 0, &viewConfigurationCount, nullptr), "Failed to enumerate View Configurations.");
        m_viewConfigurations.resize(viewConfigurationCount);
        OPENVR_CHECK(xrEnumerateViewConfigurations(vrInstance.get(), m_systemID, viewConfigurationCount, &viewConfigurationCount, m_viewConfigurations.data()), "Failed to enumerate View Configurations.");

// Pick the first application supported View Configuration Type con supported by the hardware.
        for (const XrViewConfigurationType &viewConfiguration : m_applicationViewConfigurations) {
            if (std::find(m_viewConfigurations.begin(), m_viewConfigurations.end(), viewConfiguration) != m_viewConfigurations.end()) {
                m_viewConfiguration = viewConfiguration;
                break;
            }
        }
        if (m_viewConfiguration == XR_VIEW_CONFIGURATION_TYPE_MAX_ENUM) {
            std::cerr << "Failed to find a view configuration type. Defaulting to XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO." << std::endl;
            m_viewConfiguration = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        }

// Gets the View Configuration Views. The first call gets the count of the array that will be returned. The next call fills out the array.
        uint32_t viewConfigurationViewCount = 0;
        OPENVR_CHECK(xrEnumerateViewConfigurationViews(vrInstance.get(), m_systemID, m_viewConfiguration, 0, &viewConfigurationViewCount, nullptr), "Failed to enumerate ViewConfiguration Views.");
        m_viewConfigurationViews.resize(viewConfigurationViewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
        OPENVR_CHECK(xrEnumerateViewConfigurationViews(vrInstance.get(), m_systemID, m_viewConfiguration, viewConfigurationViewCount, &viewConfigurationViewCount, m_viewConfigurationViews.data()), "Failed to enumerate ViewConfiguration Views.");
    }
    void CreateSwapchains()
    {
        // Get the supported swapchain formats as an array of int64_t and ordered by runtime preference.
        uint32_t formatCount = 0;
        OPENVR_CHECK(xrEnumerateSwapchainFormats(m_session, 0, &formatCount, nullptr), "Failed to enumerate Swapchain Formats");
        std::vector<int64_t> formats(formatCount);
        OPENVR_CHECK(xrEnumerateSwapchainFormats(m_session, formatCount, &formatCount, formats.data()), "Failed to enumerate Swapchain Formats");
        if (m_graphicsAPI->SelectDepthSwapchainFormat(formats) == 0) {
            std::cerr << "Failed to find depth format for Swapchain." << std::endl;
            DEBUG_BREAK;
        }
//Resize the SwapchainInfo to match the number of view in the View Configuration.
        m_colorSwapchainInfos.resize(m_viewConfigurationViews.size());
        m_depthSwapchainInfos.resize(m_viewConfigurationViews.size());

        for (size_t i = 0; i < m_viewConfigurationViews.size(); i++) {
            SwapchainInfo &colorSwapchainInfo = m_colorSwapchainInfos[i];
            SwapchainInfo &depthSwapchainInfo = m_depthSwapchainInfos[i];

            // Fill out an XrSwapchainCreateInfo structure and create an XrSwapchain.
// Color.
            XrSwapchainCreateInfo swapchainCI{XR_TYPE_SWAPCHAIN_CREATE_INFO};
            swapchainCI.createFlags = 0;
            swapchainCI.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
            swapchainCI.format = m_graphicsAPI->SelectColorSwapchainFormat(formats);                // Use GraphicsAPI to select the first compatible format.
            swapchainCI.sampleCount = m_viewConfigurationViews[i].recommendedSwapchainSampleCount;  // Use the recommended values from the XrViewConfigurationView.
            swapchainCI.width = m_viewConfigurationViews[i].recommendedImageRectWidth;
            swapchainCI.height = m_viewConfigurationViews[i].recommendedImageRectHeight;
            swapchainCI.faceCount = 1;
            swapchainCI.arraySize = 1;
            swapchainCI.mipCount = 1;
            OPENVR_CHECK(xrCreateSwapchain(m_session, &swapchainCI, &colorSwapchainInfo.swapchain), "Failed to create Color Swapchain");
            colorSwapchainInfo.swapchainFormat = swapchainCI.format;  // Save the swapchain format for later use.

            // Depth.
            swapchainCI.createFlags = 0;
            swapchainCI.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
            swapchainCI.format = m_graphicsAPI->SelectDepthSwapchainFormat(formats);                // Use GraphicsAPI to select the first compatible format.
            swapchainCI.sampleCount = m_viewConfigurationViews[i].recommendedSwapchainSampleCount;  // Use the recommended values from the XrViewConfigurationView.
            swapchainCI.width = m_viewConfigurationViews[i].recommendedImageRectWidth;
            swapchainCI.height = m_viewConfigurationViews[i].recommendedImageRectHeight;
            swapchainCI.faceCount = 1;
            swapchainCI.arraySize = 1;
            swapchainCI.mipCount = 1;
            OPENVR_CHECK(xrCreateSwapchain(m_session, &swapchainCI, &depthSwapchainInfo.swapchain), "Failed to create Depth Swapchain");
            depthSwapchainInfo.swapchainFormat = swapchainCI.format;  // Save the swapchain format for later use.

            // Get the number of images in the color/depth swapchain and allocate Swapchain image data via GraphicsAPI to store the returned array.
            uint32_t colorSwapchainImageCount = 0;
            OPENVR_CHECK(xrEnumerateSwapchainImages(colorSwapchainInfo.swapchain, 0, &colorSwapchainImageCount, nullptr), "Failed to enumerate Color Swapchain Images.");
            XrSwapchainImageBaseHeader *colorSwapchainImages = m_graphicsAPI->AllocateSwapchainImageData(colorSwapchainInfo.swapchain, GraphicsAPI::SwapchainType::COLOR, colorSwapchainImageCount);
            OPENVR_CHECK(xrEnumerateSwapchainImages(colorSwapchainInfo.swapchain, colorSwapchainImageCount, &colorSwapchainImageCount, colorSwapchainImages), "Failed to enumerate Color Swapchain Images.");

            uint32_t depthSwapchainImageCount = 0;
            OPENVR_CHECK(xrEnumerateSwapchainImages(depthSwapchainInfo.swapchain, 0, &depthSwapchainImageCount, nullptr), "Failed to enumerate Depth Swapchain Images.");
            XrSwapchainImageBaseHeader *depthSwapchainImages = m_graphicsAPI->AllocateSwapchainImageData(depthSwapchainInfo.swapchain, GraphicsAPI::SwapchainType::DEPTH, depthSwapchainImageCount);
            OPENVR_CHECK(xrEnumerateSwapchainImages(depthSwapchainInfo.swapchain, depthSwapchainImageCount, &depthSwapchainImageCount, depthSwapchainImages), "Failed to enumerate Depth Swapchain Images.");

            // Per image in the swapchains, fill out a GraphicsAPI::ImageViewCreateInfo structure and create a color/depth image view.
            for (uint32_t j = 0; j < colorSwapchainImageCount; j++) {
                GraphicsAPI::ImageViewCreateInfo imageViewCI;
                imageViewCI.image = m_graphicsAPI->GetSwapchainImage(colorSwapchainInfo.swapchain, j);
                imageViewCI.type = GraphicsAPI::ImageViewCreateInfo::Type::RTV;
                imageViewCI.view = GraphicsAPI::ImageViewCreateInfo::View::TYPE_2D;
                imageViewCI.format = colorSwapchainInfo.swapchainFormat;
                imageViewCI.aspect = GraphicsAPI::ImageViewCreateInfo::Aspect::COLOR_BIT;
                imageViewCI.baseMipLevel = 0;
                imageViewCI.levelCount = 1;
                imageViewCI.baseArrayLayer = 0;
                imageViewCI.layerCount = 1;
                colorSwapchainInfo.imageViews.push_back(m_graphicsAPI->CreateImageView(imageViewCI));
            }
            for (uint32_t j = 0; j < depthSwapchainImageCount; j++) {
                GraphicsAPI::ImageViewCreateInfo imageViewCI;
                imageViewCI.image = m_graphicsAPI->GetSwapchainImage(depthSwapchainInfo.swapchain, j);
                imageViewCI.type = GraphicsAPI::ImageViewCreateInfo::Type::DSV;
                imageViewCI.view = GraphicsAPI::ImageViewCreateInfo::View::TYPE_2D;
                imageViewCI.format = depthSwapchainInfo.swapchainFormat;
                imageViewCI.aspect = GraphicsAPI::ImageViewCreateInfo::Aspect::DEPTH_BIT;
                imageViewCI.baseMipLevel = 0;
                imageViewCI.levelCount = 1;
                imageViewCI.baseArrayLayer = 0;
                imageViewCI.layerCount = 1;
                depthSwapchainInfo.imageViews.push_back(m_graphicsAPI->CreateImageView(imageViewCI));
            }

        }



    }



    void DestroySwapchains()
    {
        // Per view in the view configuration:
        for (size_t i = 0; i < m_viewConfigurationViews.size(); i++) {
            SwapchainInfo &colorSwapchainInfo = m_colorSwapchainInfos[i];
            SwapchainInfo &depthSwapchainInfo = m_depthSwapchainInfos[i];

            // Destroy the color and depth image views from GraphicsAPI.
            for (void *&imageView : colorSwapchainInfo.imageViews) {
                m_graphicsAPI->DestroyImageView(imageView);
            }
            for (void *&imageView : depthSwapchainInfo.imageViews) {
                m_graphicsAPI->DestroyImageView(imageView);
            }

            // Free the Swapchain Image Data.
            m_graphicsAPI->FreeSwapchainImageData(colorSwapchainInfo.swapchain);
            m_graphicsAPI->FreeSwapchainImageData(depthSwapchainInfo.swapchain);

            // Destroy the swapchains.
            OPENVR_CHECK(xrDestroySwapchain(colorSwapchainInfo.swapchain), "Failed to destroy Color Swapchain");
            OPENVR_CHECK(xrDestroySwapchain(depthSwapchainInfo.swapchain), "Failed to destroy Depth Swapchain");
        }
    }

    void GetEnvironmentBlendModes()
    {
        // Retrieves the available blend modes. The first call gets the count of the array that will be returned. The next call fills out the array.
        uint32_t environmentBlendModeCount = 0;
        OPENVR_CHECK(xrEnumerateEnvironmentBlendModes(vrInstance.get(), m_systemID, m_viewConfiguration, 0, &environmentBlendModeCount, nullptr), "Failed to enumerate EnvironmentBlend Modes.");
        m_environmentBlendModes.resize(environmentBlendModeCount);
        OPENVR_CHECK(xrEnumerateEnvironmentBlendModes(vrInstance.get(), m_systemID, m_viewConfiguration, environmentBlendModeCount, &environmentBlendModeCount, m_environmentBlendModes.data()), "Failed to enumerate EnvironmentBlend Modes.");

// Pick the first application supported blend mode supported by the hardware.
        for (const XrEnvironmentBlendMode &environmentBlendMode : m_applicationEnvironmentBlendModes) {
            if (std::find(m_environmentBlendModes.begin(), m_environmentBlendModes.end(), environmentBlendMode) != m_environmentBlendModes.end()) {

                m_environmentBlendMode = environmentBlendMode;

                break;
            }
        }
        if (m_environmentBlendMode == XR_ENVIRONMENT_BLEND_MODE_MAX_ENUM) {
            XR_TUT_LOG_ERROR("Failed to find a compatible blend mode. Defaulting to XR_ENVIRONMENT_BLEND_MODE_OPAQUE.");
            m_environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        }
    }
    void CreateReferenceSpace()
    {
        // Fill out an XrReferenceSpaceCreateInfo structure and create a reference XrSpace, specifying a Local space with an identity pose as the origin.
        XrReferenceSpaceCreateInfo referenceSpaceCI{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        referenceSpaceCI.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
        referenceSpaceCI.poseInReferenceSpace = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
        OPENVR_CHECK(xrCreateReferenceSpace(m_session, &referenceSpaceCI, &m_localSpace), "Failed to create ReferenceSpace.");
    }
    void DestroyReferenceSpace()
    {
        // Destroy the reference XrSpace.
        OPENVR_CHECK(xrDestroySpace(m_localSpace), "Failed to destroy Space.")
    }
    void RenderFrame()
    {
        XrFrameState frameState{XR_TYPE_FRAME_STATE};
        XrFrameWaitInfo frameWaitInfo{XR_TYPE_FRAME_WAIT_INFO};
        OPENVR_CHECK(xrWaitFrame(m_session, &frameWaitInfo, &frameState), "Failed to wait for XR Frame.");

        XrFrameBeginInfo frameBeginInfo{XR_TYPE_FRAME_BEGIN_INFO};
        OPENVR_CHECK(xrBeginFrame(m_session, &frameBeginInfo), "Failed to begin the XR Frame.");

        // This can be local
        RenderLayerInfo renderLayerInfo{};
        renderLayerInfo.predictedDisplayTime = frameState.predictedDisplayTime;

        float dt = 1.0f / 72.0f; // fallback for first frame
        if (m_lastPredictedDisplayTime != 0) {
            XrTime delta = frameState.predictedDisplayTime - m_lastPredictedDisplayTime;
            dt = (float)delta * 1e-9f; // nanoseconds -> seconds
            if (dt <= 0.0f || dt > 1.0f) {
                dt = 1.0f / 72.0f;
            }
        }
        m_lastPredictedDisplayTime = frameState.predictedDisplayTime;

        bool sessionActive =
                (m_sessionState == XR_SESSION_STATE_SYNCHRONIZED ||
                 m_sessionState == XR_SESSION_STATE_VISIBLE ||
                 m_sessionState == XR_SESSION_STATE_FOCUSED);

        std::vector<const XrCompositionLayerBaseHeader*> layers;
        XrCompositionLayerPassthroughFB ptLayer{XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB};
        if (sessionActive && frameState.shouldRender) {
            PollActions(frameState.predictedDisplayTime);

            // New drum-stick logic
            UpdateSticks(dt);
            UpdateDrumHits(dt);

            // Keep your old cube interaction (for now)
            BlockInteraction();

            bool rendered = RenderLayer(renderLayerInfo);


            // --- Passthrough layer ---
            if (m_passthroughLayer != XR_NULL_HANDLE) {

                ptLayer.next        = nullptr;
                ptLayer.flags       = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
                ptLayer.space       = XR_NULL_HANDLE;          // <<< IMPORTANT
                ptLayer.layerHandle = m_passthroughLayer;

                // Put passthrough FIRST so it becomes the background
                layers.push_back(
                        reinterpret_cast<const XrCompositionLayerBaseHeader*>(&ptLayer)
                );

                // Also keep ptLayer alive until xrEndFrame returns
                // (that's why it's a local variable in this scope)
            }

            if (rendered) {
                layers.push_back(
                        reinterpret_cast<const XrCompositionLayerBaseHeader*>(&renderLayerInfo.layerProjection)
                );
            }
        }

        XrFrameEndInfo frameEndInfo{XR_TYPE_FRAME_END_INFO};
        frameEndInfo.displayTime = frameState.predictedDisplayTime;

        // --- 2. Meta requires OPAQUE here ---
        frameEndInfo.environmentBlendMode = m_environmentBlendMode;

        frameEndInfo.layerCount = static_cast<uint32_t>(layers.size());
        frameEndInfo.layers     = layers.empty() ? nullptr : layers.data();

        OPENVR_CHECK(xrEndFrame(m_session, &frameEndInfo), "Failed to end the XR Frame.");
    }



    bool RenderLayer(RenderLayerInfo& renderLayerInfo)
    {
        // Locate the views from the view configuration within the (reference) space at the display time.
        std::vector<XrView> views(m_viewConfigurationViews.size(), {XR_TYPE_VIEW});

        XrViewState viewState{XR_TYPE_VIEW_STATE};  // Will contain information on whether the position and/or orientation is valid and/or tracked.
        XrViewLocateInfo viewLocateInfo{XR_TYPE_VIEW_LOCATE_INFO};
        viewLocateInfo.viewConfigurationType = m_viewConfiguration;
        viewLocateInfo.displayTime = renderLayerInfo.predictedDisplayTime;
        viewLocateInfo.space = m_localSpace;
        uint32_t viewCount = 0;
        XrResult result = xrLocateViews(m_session, &viewLocateInfo, &viewState, static_cast<uint32_t>(views.size()), &viewCount, views.data());
        if (result != XR_SUCCESS) {
            XR_TUT_LOG("Failed to locate Views.");
            return false;
        }

        // Resize the layer projection views to match the view count. The layer projection views are used in the layer projection.
        renderLayerInfo.layerProjectionViews.resize(viewCount, {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});
        renderLayerInfo.layerDepthInfos.resize(viewCount, {XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR});
        // Per view in the view configuration:
        for (uint32_t i = 0; i < viewCount; i++) {
            SwapchainInfo &colorSwapchainInfo = m_colorSwapchainInfos[i];
            SwapchainInfo &depthSwapchainInfo = m_depthSwapchainInfos[i];

            // Acquire and wait for an image from the swapchains.
            // Get the image index of an image in the swapchains.
            // The timeout is infinite.
            uint32_t colorImageIndex = 0;
            uint32_t depthImageIndex = 0;
            XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
            OPENVR_CHECK(xrAcquireSwapchainImage(colorSwapchainInfo.swapchain, &acquireInfo, &colorImageIndex), "Failed to acquire Image from the Color Swapchian");
            OPENVR_CHECK(xrAcquireSwapchainImage(depthSwapchainInfo.swapchain, &acquireInfo, &depthImageIndex), "Failed to acquire Image from the Depth Swapchian");
            XrSwapchainImageWaitInfo waitInfo = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
            waitInfo.timeout = XR_INFINITE_DURATION;
            OPENVR_CHECK(xrWaitSwapchainImage(colorSwapchainInfo.swapchain, &waitInfo), "Failed to wait for Image from the Color Swapchain");
            OPENVR_CHECK(xrWaitSwapchainImage(depthSwapchainInfo.swapchain, &waitInfo), "Failed to wait for Image from the Depth Swapchain");

            // Get the width and height and construct the viewport and scissors.
            const uint32_t &width = m_viewConfigurationViews[i].recommendedImageRectWidth;
            const uint32_t &height = m_viewConfigurationViews[i].recommendedImageRectHeight;
            GraphicsAPI::Viewport viewport = {0.0f, 0.0f, (float)width, (float)height, 0.0f, 1.0f};
            GraphicsAPI::Rect2D scissor = {{(int32_t)0, (int32_t)0}, {width, height}};
            float nearZ = 0.05f;
            float farZ = 100.0f;

            // Fill out the XrCompositionLayerProjectionView structure specifying the pose and fov from the view.
            // This also associates the swapchain image with this layer projection view.
            renderLayerInfo.layerProjectionViews[i] = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
            renderLayerInfo.layerProjectionViews[i].pose = views[i].pose;
            renderLayerInfo.layerProjectionViews[i].fov = views[i].fov;
            renderLayerInfo.layerProjectionViews[i].subImage.swapchain = colorSwapchainInfo.swapchain;
            renderLayerInfo.layerProjectionViews[i].subImage.imageRect.offset.x = 0;
            renderLayerInfo.layerProjectionViews[i].subImage.imageRect.offset.y = 0;
            renderLayerInfo.layerProjectionViews[i].subImage.imageRect.extent.width = static_cast<int32_t>(width);
            renderLayerInfo.layerProjectionViews[i].subImage.imageRect.extent.height = static_cast<int32_t>(height);
            renderLayerInfo.layerProjectionViews[i].subImage.imageArrayIndex = 0;  // Useful for multiview rendering.
            renderLayerInfo.layerProjectionViews[i].next = &renderLayerInfo.layerDepthInfos[i];

            renderLayerInfo.layerDepthInfos[i] = {XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR};
            renderLayerInfo.layerDepthInfos[i].subImage.swapchain = depthSwapchainInfo.swapchain;
            renderLayerInfo.layerDepthInfos[i].subImage.imageRect.offset.x = 0;
            renderLayerInfo.layerDepthInfos[i].subImage.imageRect.offset.y = 0;
            renderLayerInfo.layerDepthInfos[i].subImage.imageRect.extent.width = static_cast<int32_t>(width);
            renderLayerInfo.layerDepthInfos[i].subImage.imageRect.extent.height = static_cast<int32_t>(height);
            renderLayerInfo.layerDepthInfos[i].minDepth = viewport.minDepth;
            renderLayerInfo.layerDepthInfos[i].maxDepth = viewport.maxDepth;
            renderLayerInfo.layerDepthInfos[i].nearZ = nearZ;
            renderLayerInfo.layerDepthInfos[i].farZ = farZ;
            // Rendering code to clear the color and depth image views.
            m_graphicsAPI->BeginRendering();

            m_graphicsAPI->ClearColor(colorSwapchainInfo.imageViews[colorImageIndex], 0.00f, 0.00f, 0.00f, 10.00f);

            m_graphicsAPI->ClearDepth(depthSwapchainInfo.imageViews[depthImageIndex], 1.0f);

            // Render Game Objects here
            m_graphicsAPI->SetRenderAttachments(&colorSwapchainInfo.imageViews[colorImageIndex], 1, depthSwapchainInfo.imageViews[depthImageIndex], width, height, m_pipeline);
            m_graphicsAPI->SetViewports(&viewport, 1);
            m_graphicsAPI->SetScissors(&scissor, 1);

// Compute the view-projection transform.
// All matrices (including OpenXR's) are column-major, right-handed.
            XrMatrix4x4f proj;
            XrMatrix4x4f_CreateProjectionFov(&proj, m_apiType, views[i].fov, nearZ, farZ);
            XrMatrix4x4f toView;
            XrVector3f scale1m{1.0f, 1.0f, 1.0f};
            XrMatrix4x4f_CreateTranslationRotationScale(&toView, &views[i].pose.position, &views[i].pose.orientation, &scale1m);
            XrMatrix4x4f view;
            XrMatrix4x4f_InvertRigidBody(&view, &toView);
            XrMatrix4x4f_Multiply(&cameraConstants.viewProj, &proj, &view);


            renderCuboidIndex = 0;
// Draw a floor. Scale it by 2 in the X and Z, and 0.1 in the Y,
       //     RenderCuboid({{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, -m_viewHeightM, 0.0f}}, {2.0f, 0.1f, 2.0f}, {0.4f, 0.5f, 0.5f});
// Draw a "table".
       //     RenderCuboid({{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, -m_viewHeightM + 0.9f, -0.7f}}, {1.0f, 0.2f, 1.0f}, {0.6f, 0.6f, 0.4f});

//            // Draw some blocks at the controller positions:
//            for (int j = 0; j < 2; j++) {
//                if (m_handPoseState[j].isActive) {
//                    RenderCuboid(m_handPose[j], {0.02f, 0.04f, 0.10f}, {1.f, 1.f, 1.f});
//                }
//            }
//            for (int j = 0; j < m_blocks.size(); j++) {
//                auto &thisBlock = m_blocks[j];
//                XrVector3f sc = thisBlock.scale;
//                if (j == m_nearBlock[0] || j == m_nearBlock[1])
//                    sc = thisBlock.scale * 1.05f;
//                RenderCuboid(thisBlock.pose, sc, thisBlock.color);
//            }

// TEMP: debug cube at (0, 0, -1)
// TEMP: red debug cube straight in front


// Visualize drums as thin pads
            for (int d = 0; d < m_drumCount; ++d) {
                XrPosef p{};
                p.orientation = {0,0,0,1};
                p.position    = m_drums[d].center;

                XrVector3f scale = { m_drums[d].radius, 0.03f, m_drums[d].radius };
                XrVector3f col   = { 0.2f, 0.8f, 0.2f }; // dark grey

                RenderCuboid(p, scale, col);
            }

// Visualize sticks as long skinny cuboids
            for (int sIdx = 0; sIdx < 2; ++sIdx) {
                Stick& s = m_sticks[sIdx];
                if (!s.active) continue;

                XrPosef pose = s.pose;
                XrVector3f sc = { 0.01f, s.length * 0.5f, 0.01f };
                XrVector3f half = Rotate(pose.orientation, {0.0f, sc.y, 0.0f});
                pose.position = pose.position + half;

                RenderCuboid(pose, sc, {0.7f, 0.6f, 0.3f}); // stick color
            }

            if (handTrackingSystemProperties.supportsHandTracking) {
                for (int j = 0; j < 2; j++) {
                    auto hand = m_hands[j];
                    XrVector3f hand_color = {1.f, 1.f, 0.f};
                    for (int k = 0; k < XR_HAND_JOINT_COUNT_EXT; k++) {
                        XrVector3f sc = {1.5f, 1.5f, 2.5f};
                        sc = sc * hand.m_jointLocations[k].radius;
                        RenderCuboid(hand.m_jointLocations[k].pose, sc, hand_color);
                    }
                }
            }

            m_graphicsAPI->EndRendering();

            // Give the swapchain image back to OpenXR, allowing the compositor to use the image.
            XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            OPENVR_CHECK(xrReleaseSwapchainImage(colorSwapchainInfo.swapchain, &releaseInfo), "Failed to release Image back to the Color Swapchain");
            OPENVR_CHECK(xrReleaseSwapchainImage(depthSwapchainInfo.swapchain, &releaseInfo), "Failed to release Image back to the Depth Swapchain");
        }

        // Fill out the XrCompositionLayerProjection structure for usage with xrEndFrame().
        renderLayerInfo.layerProjection.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT | XR_COMPOSITION_LAYER_CORRECT_CHROMATIC_ABERRATION_BIT;
        renderLayerInfo.layerProjection.space = m_localSpace;
        renderLayerInfo.layerProjection.viewCount = static_cast<uint32_t>(renderLayerInfo.layerProjectionViews.size());
        renderLayerInfo.layerProjection.views = renderLayerInfo.layerProjectionViews.data();

        return true;
    }

    size_t renderCuboidIndex = 0;
    void RenderCuboid(XrPosef pose, XrVector3f scale, XrVector3f color)
    {
        XrMatrix4x4f_CreateTranslationRotationScale(&cameraConstants.model, &pose.position, &pose.orientation, &scale);

        XrMatrix4x4f_Multiply(&cameraConstants.modelViewProj, &cameraConstants.viewProj, &cameraConstants.model);
        cameraConstants.color = {color.x, color.y, color.z, 1.0};
        size_t offsetCameraUB = sizeof(CameraConstants) * renderCuboidIndex;

        m_graphicsAPI->SetPipeline(m_pipeline);

        m_graphicsAPI->SetBufferData(m_uniformBuffer_Camera, offsetCameraUB, sizeof(CameraConstants), &cameraConstants);
        m_graphicsAPI->SetDescriptor({0, m_uniformBuffer_Camera, GraphicsAPI::DescriptorInfo::Type::BUFFER, GraphicsAPI::DescriptorInfo::Stage::VERTEX, false, offsetCameraUB, sizeof(CameraConstants)});
        m_graphicsAPI->SetDescriptor({1, m_uniformBuffer_Normals, GraphicsAPI::DescriptorInfo::Type::BUFFER, GraphicsAPI::DescriptorInfo::Stage::VERTEX, false, 0, sizeof(normals)});

        m_graphicsAPI->UpdateDescriptors();

        m_graphicsAPI->SetVertexBuffers(&m_vertexBuffer, 1);
        m_graphicsAPI->SetIndexBuffer(m_indexBuffer);
        m_graphicsAPI->DrawIndexed(36);

        renderCuboidIndex++;
    }

    struct CameraConstants {
        XrMatrix4x4f viewProj;
        XrMatrix4x4f modelViewProj;
        XrMatrix4x4f model;
        XrVector4f color;
        XrVector4f pad1;
        XrVector4f pad2;
        XrVector4f pad3;
    };
    CameraConstants cameraConstants;
    XrVector4f normals[6] = {
            {1.00f, 0.00f, 0.00f, 0},
            {-1.00f, 0.00f, 0.00f, 0},
            {0.00f, 1.00f, 0.00f, 0},
            {0.00f, -1.00f, 0.00f, 0},
            {0.00f, 0.00f, 1.00f, 0},
            {0.00f, 0.0f, -1.00f, 0}};
    void CreateResources()
    {
        // Vertices for a 1x1x1 meter cube. (Left/Right, Top/Bottom, Front/Back)
        constexpr XrVector4f vertexPositions[] = {
                {+0.5f, +0.5f, +0.5f, 1.0f},
                {+0.5f, +0.5f, -0.5f, 1.0f},
                {+0.5f, -0.5f, +0.5f, 1.0f},
                {+0.5f, -0.5f, -0.5f, 1.0f},
                {-0.5f, +0.5f, +0.5f, 1.0f},
                {-0.5f, +0.5f, -0.5f, 1.0f},
                {-0.5f, -0.5f, +0.5f, 1.0f},
                {-0.5f, -0.5f, -0.5f, 1.0f}};

#define CUBE_FACE(V1, V2, V3, V4, V5, V6) vertexPositions[V1], vertexPositions[V2], vertexPositions[V3], vertexPositions[V4], vertexPositions[V5], vertexPositions[V6],

        XrVector4f cubeVertices[] = {
                CUBE_FACE(2, 1, 0, 2, 3, 1)  // -X
                CUBE_FACE(6, 4, 5, 6, 5, 7)  // +X
                CUBE_FACE(0, 1, 5, 0, 5, 4)  // -Y
                CUBE_FACE(2, 6, 7, 2, 7, 3)  // +Y
                CUBE_FACE(0, 4, 6, 0, 6, 2)  // -Z
                CUBE_FACE(1, 3, 7, 1, 7, 5)  // +Z
        };

        uint32_t cubeIndices[36] = {
                0, 1, 2, 3, 4, 5,        // -X
                6, 7, 8, 9, 10, 11,      // +X
                12, 13, 14, 15, 16, 17,  // -Y
                18, 19, 20, 21, 22, 23,  // +Y
                24, 25, 26, 27, 28, 29,  // -Z
                30, 31, 32, 33, 34, 35,  // +Z
        };

        m_vertexBuffer = m_graphicsAPI->CreateBuffer({GraphicsAPI::BufferCreateInfo::Type::VERTEX, sizeof(float) * 4, sizeof(cubeVertices), &cubeVertices});

        m_indexBuffer = m_graphicsAPI->CreateBuffer({GraphicsAPI::BufferCreateInfo::Type::INDEX, sizeof(uint32_t), sizeof(cubeIndices), &cubeIndices});

        size_t numberOfCuboids = m_maxBlockCount + 2 + 2;
        numberOfCuboids += XR_HAND_JOINT_COUNT_EXT * 2;
        m_uniformBuffer_Camera = m_graphicsAPI->CreateBuffer({GraphicsAPI::BufferCreateInfo::Type::UNIFORM, 0, sizeof(CameraConstants) * numberOfCuboids, nullptr});
        m_uniformBuffer_Normals = m_graphicsAPI->CreateBuffer({GraphicsAPI::BufferCreateInfo::Type::UNIFORM, 0, sizeof(normals), &normals});

        // Create Shaders
        if (m_apiType == OPENGL_ES) {
            std::string vertexSource = ReadTextFile("shaders/VertexShader_GLES.glsl", androidApp->activity->assetManager);
            m_vertexShader = m_graphicsAPI->CreateShader({GraphicsAPI::ShaderCreateInfo::Type::VERTEX, vertexSource.data(), vertexSource.size()});
            std::string fragmentSource = ReadTextFile("shaders/PixelShader_GLES.glsl", androidApp->activity->assetManager);
            m_fragmentShader = m_graphicsAPI->CreateShader({GraphicsAPI::ShaderCreateInfo::Type::FRAGMENT, fragmentSource.data(), fragmentSource.size()});
        }

        // create pipeline for drawing a solid cube
        GraphicsAPI::PipelineCreateInfo pipelineCI;
        pipelineCI.shaders = {m_vertexShader, m_fragmentShader};
        pipelineCI.vertexInputState.attributes = {{0, 0, GraphicsAPI::VertexType::VEC4, 0, "TEXCOORD"}};
        pipelineCI.vertexInputState.bindings = {{0, 0, 4 * sizeof(float)}};
        pipelineCI.inputAssemblyState = {GraphicsAPI::PrimitiveTopology::TRIANGLE_LIST, false};
        pipelineCI.rasterisationState = {false, false, GraphicsAPI::PolygonMode::FILL, GraphicsAPI::CullMode::BACK, GraphicsAPI::FrontFace::COUNTER_CLOCKWISE, false, 0.0f, 0.0f, 0.0f, 1.0f};
        pipelineCI.multisampleState = {1, false, 1.0f, 0xFFFFFFFF, false, false};
        pipelineCI.depthStencilState = {true, true, GraphicsAPI::CompareOp::LESS_OR_EQUAL, false, false, {}, {}, 0.0f, 1.0f};
        pipelineCI.colorBlendState = {false, GraphicsAPI::LogicOp::NO_OP, {{true, GraphicsAPI::BlendFactor::SRC_ALPHA, GraphicsAPI::BlendFactor::ONE_MINUS_SRC_ALPHA, GraphicsAPI::BlendOp::ADD, GraphicsAPI::BlendFactor::ONE, GraphicsAPI::BlendFactor::ZERO, GraphicsAPI::BlendOp::ADD, (GraphicsAPI::ColorComponentBit)15}}, {0.0f, 0.0f, 0.0f, 0.0f}};
        pipelineCI.colorFormats = {m_colorSwapchainInfos[0].swapchainFormat};
        pipelineCI.depthFormat = m_depthSwapchainInfos[0].swapchainFormat;
        pipelineCI.layout = {{0, nullptr, GraphicsAPI::DescriptorInfo::Type::BUFFER, GraphicsAPI::DescriptorInfo::Stage::VERTEX},
                             {1, nullptr, GraphicsAPI::DescriptorInfo::Type::BUFFER, GraphicsAPI::DescriptorInfo::Stage::VERTEX},
                             {2, nullptr, GraphicsAPI::DescriptorInfo::Type::BUFFER, GraphicsAPI::DescriptorInfo::Stage::FRAGMENT}};
        m_pipeline = m_graphicsAPI->CreatePipeline(pipelineCI);

        // Create sixty-four cubic blocks, 20cm wide, evenly distributed,
// and randomly colored.
        float scale = 0.2f;
// Center the blocks a little way from the origin.
        XrVector3f center = {0.0f, -0.2f, -0.7f};
        for (int i = 0; i < 4; i++) {
            float x = scale * (float(i) - 1.5f) + center.x;
            for (int j = 0; j < 4; j++) {
                float y = scale * (float(j) - 1.5f) + center.y;
                for (int k = 0; k < 4; k++) {
                    float angleRad = 0;
                    float z = scale * (float(k) - 1.5f) + center.z;
                    // No rotation
                    XrQuaternionf q = {0.0f, 0.0f, 0.0f, 1.0f};
                    // A random color.
                    XrVector3f color = {pseudorandom_distribution(pseudo_random_generator), pseudorandom_distribution(pseudo_random_generator), pseudorandom_distribution(pseudo_random_generator)};
                    m_blocks.push_back({{q, {x, y, z}}, {0.095f, 0.095f, 0.095f}, color});
                    if (m_blocks.size() > m_maxBlockCount) {
                        m_blocks.pop_front();
                    }
                }
            }
        }

        AAssetManager* assetMgr = VrApp::androidApp->activity->assetManager;
        if (!m_audioEngine.init(assetMgr, "sfx/snare.wav")) {
            __android_log_print(ANDROID_LOG_ERROR, "VrApp", "Failed to init AudioEngine with snare.wav");
        }


        InitDrumsAndSticks();
    }

    void DestroyResources()
    {

        m_audioEngine.shutdown();

        m_graphicsAPI->DestroyPipeline(m_pipeline);
        m_graphicsAPI->DestroyShader(m_fragmentShader);
        m_graphicsAPI->DestroyShader(m_vertexShader);
        m_graphicsAPI->DestroyBuffer(m_uniformBuffer_Camera);
        m_graphicsAPI->DestroyBuffer(m_uniformBuffer_Normals);
        m_graphicsAPI->DestroyBuffer(m_indexBuffer);
        m_graphicsAPI->DestroyBuffer(m_vertexBuffer);
    }

    void PollEvents();
    void PollActions(XrTime predictedTime) {
        // Update our action set with up-to-date input data.
        // First, we specify the actionSet we are polling.
        XrActiveActionSet activeActionSet{};
        activeActionSet.actionSet = m_actionSet;
        activeActionSet.subactionPath = XR_NULL_PATH;
        // Now we sync the Actions to make sure they have current data.
        XrActionsSyncInfo actionsSyncInfo{XR_TYPE_ACTIONS_SYNC_INFO};
        actionsSyncInfo.countActiveActionSets = 1;
        actionsSyncInfo.activeActionSets = &activeActionSet;
        OPENVR_CHECK(xrSyncActions(m_session, &actionsSyncInfo), "Failed to sync Actions.");

        XrActionStateGetInfo actionStateGetInfo{XR_TYPE_ACTION_STATE_GET_INFO};
        // We pose a single Action, twice - once for each subAction Path.
        actionStateGetInfo.action = m_palmPoseAction;
        // For each hand, get the pose state if possible.
        for (int i = 0; i < 2; i++) {
            // Specify the subAction Path.
            actionStateGetInfo.subactionPath = m_handPaths[i];
            OPENVR_CHECK(xrGetActionStatePose(m_session, &actionStateGetInfo, &m_handPoseState[i]), "Failed to get Pose State.");
            if (m_handPoseState[i].isActive) {
                XrSpaceLocation spaceLocation{XR_TYPE_SPACE_LOCATION};
                XrResult res = xrLocateSpace(m_handPoseSpace[i], m_localSpace, predictedTime, &spaceLocation);
                if (XR_UNQUALIFIED_SUCCESS(res) &&
                    (spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                    (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                    m_handPose[i] = spaceLocation.pose;
                } else {
                    m_handPoseState[i].isActive = false;
                }
            }


        }

        for (int i = 0; i < 2; i++) {
            actionStateGetInfo.action = m_grabCubeAction;
            actionStateGetInfo.subactionPath = m_handPaths[i];
            OPENVR_CHECK(xrGetActionStateFloat(m_session, &actionStateGetInfo, &m_grabState[i]), "Failed to get Float State of Grab Cube action.");
        }
        for (int i = 0; i < 2; i++) {
            actionStateGetInfo.action = m_changeColorAction;
            actionStateGetInfo.subactionPath = m_handPaths[i];
            OPENVR_CHECK(xrGetActionStateBoolean(m_session, &actionStateGetInfo, &m_changeColorState[i]), "Failed to get Boolean State of Change Color action.");
        }
        // The Spawn Cube action has no subActionPath:
        {
            actionStateGetInfo.action = m_spawnCubeAction;
            actionStateGetInfo.subactionPath = 0;
            OPENVR_CHECK(xrGetActionStateBoolean(m_session, &actionStateGetInfo, &m_spawnCubeState), "Failed to get Boolean State of Spawn Cube action.");
        }

        for (int i = 0; i < 2; i++) {
            m_buzz[i] *= 0.5f;
            if (m_buzz[i] < 0.01f)
                m_buzz[i] = 0.0f;
            XrHapticVibration vibration{XR_TYPE_HAPTIC_VIBRATION};
            vibration.amplitude = m_buzz[i];
            vibration.duration = XR_MIN_HAPTIC_DURATION;
            vibration.frequency = XR_FREQUENCY_UNSPECIFIED;

            XrHapticActionInfo hapticActionInfo{XR_TYPE_HAPTIC_ACTION_INFO};
            hapticActionInfo.action = m_buzzAction;
            hapticActionInfo.subactionPath = m_handPaths[i];
            OPENVR_CHECK(xrApplyHapticFeedback(m_session, &hapticActionInfo, (XrHapticBaseHeader *)&vibration), "Failed to apply haptic feedback.");
        }

        if (handTrackingSystemProperties.supportsHandTracking) {
            XrActionStateGetInfo getInfo{XR_TYPE_ACTION_STATE_GET_INFO};
            for (int i = 0; i < 2; i++) {
                bool Unobstructed = true;
                Hand &hand = m_hands[i];
                XrHandJointsMotionRangeInfoEXT motionRangeInfo{XR_TYPE_HAND_JOINTS_MOTION_RANGE_INFO_EXT};
                motionRangeInfo.handJointsMotionRange = Unobstructed
                                                        ? XR_HAND_JOINTS_MOTION_RANGE_UNOBSTRUCTED_EXT
                                                        : XR_HAND_JOINTS_MOTION_RANGE_CONFORMING_TO_CONTROLLER_EXT;
                XrHandJointsLocateInfoEXT locateInfo{XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT, &motionRangeInfo};
                locateInfo.baseSpace = m_localSpace;
                locateInfo.time = predictedTime;

                XrHandJointLocationsEXT locations{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
                locations.jointCount = (uint32_t)XR_HAND_JOINT_COUNT_EXT;
                locations.jointLocations = hand.m_jointLocations;
                OPENVR_CHECK(xrLocateHandJointsEXT(hand.m_handTracker, &locateInfo, &locations), "Failed to locate hand joints.");
            }
        }

    }

    // Helper function to snap a 3D position to the nearest 10cm
    static XrVector3f FixPosition(XrVector3f pos) {
        int x = int(std::nearbyint(pos.x * 10.f));
        int y = int(std::nearbyint(pos.y * 10.f));
        int z = int(std::nearbyint(pos.z * 10.f));
        pos.x = float(x) / 10.f;
        pos.y = float(y) / 10.f;
        pos.z = float(z) / 10.f;
        return pos;
    }
// Handle the interaction between the user's hands, the grab action, and the 3D blocks.
    void BlockInteraction() {
        // For each hand:
        for (int i = 0; i < 2; i++) {
            float nearest = 1.0f;
            // If not currently holding a block:
            if (m_grabbedBlock[i] == -1) {
                m_nearBlock[i] = -1;
                // Only if the pose was detected this frame:
                if (m_handPoseState[i].isActive) {
                    // For each block:
                    for (int j = 0; j < m_blocks.size(); j++) {
                        auto block = m_blocks[j];
                        // How far is it from the hand to this block?
                        XrVector3f diff = block.pose.position - m_handPose[i].position;
                        float distance = std::max(fabs(diff.x), std::max(fabs(diff.y), fabs(diff.z)));
                        if (distance < 0.05f && distance < nearest) {
                            m_nearBlock[i] = j;
                            nearest = distance;
                        }
                    }
                }
                if (m_nearBlock[i] != -1) {
                    if (m_grabState[i].isActive && m_grabState[i].currentState > 0.5f) {
                        m_grabbedBlock[i] = m_nearBlock[i];
                        m_buzz[i] = 1.0f;
                    } else if (m_changeColorState[i].isActive == XR_TRUE && m_changeColorState[i].currentState == XR_FALSE && m_changeColorState[i].changedSinceLastSync == XR_TRUE) {
                        auto &thisBlock = m_blocks[m_nearBlock[i]];
                        XrVector3f color = {pseudorandom_distribution(pseudo_random_generator), pseudorandom_distribution(pseudo_random_generator), pseudorandom_distribution(pseudo_random_generator)};
                        thisBlock.color = color;
                    }
                } else {
                    // right hand not near a block?
                    // i = 1 means sub action path /user/hand/right
                    if (i == 1 && m_spawnCubeState.isActive == XR_TRUE && m_spawnCubeState.currentState == XR_FALSE && m_spawnCubeState.changedSinceLastSync == XR_TRUE) {
                        XrQuaternionf q = {0.0f, 0.0f, 0.0f, 1.0f};
                        XrVector3f color = {pseudorandom_distribution(pseudo_random_generator), pseudorandom_distribution(pseudo_random_generator), pseudorandom_distribution(pseudo_random_generator)};
                        m_blocks.push_back({{q, FixPosition(m_handPose[i].position)}, {0.095f, 0.095f, 0.095f}, color});
                        while ( m_blocks.size() > m_maxBlockCount) {
                            m_blocks.pop_front();
                        }
                    }
                }
            } else {
                m_nearBlock[i] = m_grabbedBlock[i];
                if (m_handPoseState[i].isActive)
                    m_blocks[m_grabbedBlock[i]].pose.position = m_handPose[i].position;
                if (!m_grabState[i].isActive || m_grabState[i].currentState < 0.5f) {
                    m_blocks[m_grabbedBlock[i]].pose.position = FixPosition(m_blocks[m_grabbedBlock[i]].pose.position);
                    m_grabbedBlock[i] = -1;
                    m_buzz[i] = 0.2f;
                }
            }
        }
    }

    void PollSystemEvents();


};

void VrApp::PollEvents()
{
// Poll OpenXR for a new event.
    XrEventDataBuffer eventData{XR_TYPE_EVENT_DATA_BUFFER};
    auto XrPollEvents = [&]() -> bool {
        eventData = {XR_TYPE_EVENT_DATA_BUFFER};
        return xrPollEvent(vrInstance.get(), &eventData) == XR_SUCCESS;
    };

    while (XrPollEvents()) {
        switch (eventData.type) {
            // Log the number of lost events from the runtime.
            case XR_TYPE_EVENT_DATA_EVENTS_LOST: {
                XrEventDataEventsLost *eventsLost = reinterpret_cast<XrEventDataEventsLost *>(&eventData);
                XR_TUT_LOG("OPENXR: Events Lost: " << eventsLost->lostEventCount);
                break;
            }
                // Log that an instance loss is pending and shutdown the application.
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
                XrEventDataInstanceLossPending *instanceLossPending = reinterpret_cast<XrEventDataInstanceLossPending *>(&eventData);
                XR_TUT_LOG("OPENXR: Instance Loss Pending at: " << instanceLossPending->lossTime);
                m_sessionRunning = false;
                m_applicationRunning = false;
                break;
            }
                // Log that the interaction profile has changed.
            case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
                XrEventDataInteractionProfileChanged *interactionProfileChanged = reinterpret_cast<XrEventDataInteractionProfileChanged *>(&eventData);
                XR_TUT_LOG("OPENXR: Interaction Profile changed for Session: " << interactionProfileChanged->session);
                if (interactionProfileChanged->session != m_session) {
                    XR_TUT_LOG("XrEventDataInteractionProfileChanged for unknown Session");
                    break;
                }
                RecordCurrentBindings();
                break;
            }
                // Log that there's a reference space change pending.
            case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
                XrEventDataReferenceSpaceChangePending *referenceSpaceChangePending = reinterpret_cast<XrEventDataReferenceSpaceChangePending *>(&eventData);
                XR_TUT_LOG("OPENXR: Reference Space Change pending for Session: " << referenceSpaceChangePending->session);
                if (referenceSpaceChangePending->session != m_session) {
                    XR_TUT_LOG("XrEventDataReferenceSpaceChangePending for unknown Session");
                    break;
                }
                break;
            }
                // Session State changes:
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                XrEventDataSessionStateChanged *sessionStateChanged = reinterpret_cast<XrEventDataSessionStateChanged *>(&eventData);
                if (sessionStateChanged->session != m_session) {
                    XR_TUT_LOG("XrEventDataSessionStateChanged for unknown Session");
                    break;
                }

                if (sessionStateChanged->state == XR_SESSION_STATE_READY) {
                    // SessionState is ready. Begin the XrSession using the XrViewConfigurationType.
                    XrSessionBeginInfo sessionBeginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                    sessionBeginInfo.primaryViewConfigurationType = m_viewConfiguration;
                    OPENVR_CHECK(xrBeginSession(m_session, &sessionBeginInfo), "Failed to begin Session.");
                    m_sessionRunning = true;
                }
                if (sessionStateChanged->state == XR_SESSION_STATE_STOPPING) {
                    // SessionState is stopping. End the XrSession.
                    OPENVR_CHECK(xrEndSession(m_session), "Failed to end Session.");
                    m_sessionRunning = false;
                }
                if (sessionStateChanged->state == XR_SESSION_STATE_EXITING) {
                    // SessionState is exiting. Exit the application.
                    m_sessionRunning = false;
                    m_applicationRunning = false;
                }
                if (sessionStateChanged->state == XR_SESSION_STATE_LOSS_PENDING) {
                    // SessionState is loss pending. Exit the application.
                    // It's possible to try a reestablish an XrInstance and XrSession, but we will simply exit here.
                    m_sessionRunning = false;
                    m_applicationRunning = false;
                }
                // Store state for reference across the application.
                m_sessionState = sessionStateChanged->state;
                break;
            }
            default: {
                break;
            }
        }
    }
}


android_app *VrApp::androidApp = nullptr;
VrApp::AndroidAppState VrApp::androidAppState = {};

void VrApp_Main(GraphicsAPI_Type apiType)
{
    DebugOutput debugOutput;
    XR_TUT_LOG("VrApp");
    VrApp app(apiType);
    app.Run();
}

void android_main(struct android_app *app)
{
    JNIEnv *env;
    app->activity->vm->AttachCurrentThread(&env, nullptr);
    XrInstance m_xrInstance = XR_NULL_HANDLE;
    PFN_xrInitializeLoaderKHR xrInitializeLoaderKHR = nullptr;
    OPENXR_CHECK(xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", (PFN_xrVoidFunction *)&xrInitializeLoaderKHR), "Failed to get InstanceProcAddr for xrInitializeLoaderKHR.");
    if (!xrInitializeLoaderKHR)
    {
        return;
    }
    XrLoaderInitInfoAndroidKHR loaderInitializeInfoAndroid{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
    loaderInitializeInfoAndroid.applicationVM = app->activity->vm;
    loaderInitializeInfoAndroid.applicationContext = app->activity->clazz;
    OPENXR_CHECK(xrInitializeLoaderKHR((XrLoaderInitInfoBaseHeaderKHR *)&loaderInitializeInfoAndroid), "Failed to initialize Loader for Android.");
    app->userData = &VrApp::androidAppState;
    app->onAppCmd = VrApp::AndroidAppHandleCmd;
    VrApp::androidApp = app;
    VrApp_Main(OPENGL_ES);
}


void VrApp::AndroidAppHandleCmd(struct android_app *app, int32_t cmd)
{
    auto appState = (VrApp::AndroidAppState *)app->userData;
    switch (cmd)
    {
        case APP_CMD_START:
        {
        }
            break;
        case APP_CMD_RESUME:
        {
            appState->resumed = true;
        }
            break;
        case APP_CMD_PAUSE:
        {
            appState->resumed = false;
        }
            break;
        case APP_CMD_STOP:
        {
        }
            break;
        case APP_CMD_DESTROY:
        {
            appState->nativeWindow = nullptr;
        }
            break;
        case APP_CMD_INIT_WINDOW:
        {
            appState->nativeWindow = app->window;
        }
            break;
        case APP_CMD_TERM_WINDOW:
        {
            appState->nativeWindow = nullptr;
        }
            break;
        default:
        {}
            break;
    }
}

void VrApp::PollSystemEvents()
{
    if (androidApp->destroyRequested != 0)
    {
        m_applicationRunning = false;
        return;
    }
    while (true)
    {
        struct android_poll_source *source = nullptr;
        int events = 0;
        const int timeoutMilliseconds = (!androidAppState.resumed && !m_sessionRunning && androidApp->destroyRequested == 0) ? -1 : 0;
        if (ALooper_pollOnce(timeoutMilliseconds, nullptr, &events, (void**)&source) >= 0)
        {
            if (source != nullptr)
            {
                source->process(androidApp, source);
            }
        }
        else
        {
            break;
        }
    }
}


