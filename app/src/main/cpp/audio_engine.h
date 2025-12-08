#ifndef VRAPP_AUDIO_ENGINE_H
#define VRAPP_AUDIO_ENGINE_H

#include <atomic>
#include <vector>
#include <cstdint>
#include <android/asset_manager.h>
#include <oboe/Oboe.h>
#include <stdint.h>
#include <__stddef_size_t.h>

enum DrumSoundId {
    SOUND_SNARE = 0,
    SOUND_HI_TOM,
    SOUND_MID_TOM,
    SOUND_FLOOR_TOM,
    SOUND_HIHAT_CLOSED,
    SOUND_HIHAT_OPEN,
    SOUND_CRASH,
    SOUND_RIDE,
    SOUND_BASS,
    SOUND_COUNT
};


class AudioEngine : public oboe::AudioStreamCallback {
public:
    AudioEngine() = default;
    ~AudioEngine() override;

    bool init(AAssetManager* assetMgr,
              const char* snareAssetPath,
              const char* hiTomAssetPath,
              const char* midTomAssetPath,
              const char* floorTomAssetPath,
              const char* closedHiHatAssetPath,
              const char* openHiHatAssetPath,
              const char* crashAssetPath,
              const char* rideAssetPath,
              const char* kickAssetPath);

    void shutdown();

    void playDrumSound(int soundId);
    // Call this when the drum-stick actually hits the drum
    void playSnare();

    // Oboe callback
    oboe::DataCallbackResult onAudioReady(
            oboe::AudioStream* audioStream,
            void* audioData,
            int32_t numFrames) override;

    void onErrorAfterClose(
            oboe::AudioStream* stream,
            oboe::Result error) override;


    bool parseWav16(
            const uint8_t* data,
            size_t size,
            std::vector<int16_t>& outSamples,
            uint16_t& outChannels,
            uint32_t& outSampleRate);
    static constexpr int kNumDrums = SOUND_COUNT;
private:
    // One sample slot per drum
    struct SampleSlot {
        std::vector<int16_t> samples;
        std::atomic<int32_t> framePlayhead{-1};  // in frames, -1 = idle
        uint16_t channels = 0;
        uint32_t sampleRate = 48000;
    };


    SampleSlot m_slots[kNumDrums];

    // bool loadSnareFromAsset(AAssetManager* mgr, const char* path);

//    // Interleaved PCM 16-bit samples (mono or stereo)
//    std::vector<int16_t> mSnareSamples;
//
//    // Playhead in *frames* (not raw samples). -1 = not playing
//    std::atomic<int32_t> mSnareFramePlayhead{-1};
//
//    // WAV format info
//    uint16_t mSnareChannels = 1;
//    uint32_t mSnareSampleRate = 48000;

    std::shared_ptr<oboe::AudioStream> mStream;
};

#endif //VRAPP_AUDIO_ENGINE_H