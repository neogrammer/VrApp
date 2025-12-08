#ifndef VRAPP_AUDIO_ENGINE_H
#define VRAPP_AUDIO_ENGINE_H

#include <atomic>
#include <vector>
#include <cstdint>
#include <android/asset_manager.h>
#include <oboe/Oboe.h>

class AudioEngine : public oboe::AudioStreamCallback {
public:
    AudioEngine() = default;
    ~AudioEngine();

    bool init(AAssetManager* assetMgr, const char* snareAssetPath);
    void shutdown();

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

private:
    bool loadSnareFromAsset(AAssetManager* mgr, const char* path);

    // Interleaved PCM 16-bit samples (mono or stereo)
    std::vector<int16_t> mSnareSamples;

    // Playhead in *frames* (not raw samples). -1 = not playing
    std::atomic<int32_t> mSnareFramePlayhead{-1};

    // WAV format info
    uint16_t mSnareChannels = 1;
    uint32_t mSnareSampleRate = 48000;

    std::shared_ptr<oboe::AudioStream> mStream;
};

#endif //VRAPP_AUDIO_ENGINE_H