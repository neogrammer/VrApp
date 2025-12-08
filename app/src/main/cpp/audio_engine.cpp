#include "audio_engine.h"
#include <android/log.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <cstring>

#define LOG_TAG "AudioEngine"
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)

AudioEngine::~AudioEngine() {
    shutdown();
}

bool AudioEngine::init(AAssetManager* assetMgr, const char* snareAssetPath) {
    if (!assetMgr) {
        LOGE("init: assetMgr is null");
        return false;
    }

    if (!loadSnareFromAsset(assetMgr, snareAssetPath)) {
        LOGE("Failed to load snare sample from asset: %s", snareAssetPath);
        return false;
    }

    oboe::AudioStreamBuilder builder;

    builder.setDirection(oboe::Direction::Output)->setSharingMode(oboe::SharingMode::Exclusive)->setPerformanceMode(oboe::PerformanceMode::LowLatency)->setFormat(oboe::AudioFormat::Float)->setChannelCount(oboe::ChannelCount::Stereo)->setCallback(this);
    oboe::Result result = builder.openStream(mStream);
    if (result != oboe::Result::OK || !mStream) {
        LOGE("openStream failed: %s", oboe::convertToText(result));
        return false;
    }

    result = mStream->requestStart();
    if (result != oboe::Result::OK) {
        LOGE("requestStart failed: %s", oboe::convertToText(result));
        mStream->close();
        mStream.reset();
        return false;
    }

    LOGI("AudioEngine initialized");
    return true;
}

void AudioEngine::shutdown() {
    if (mStream) {
        mStream->stop();
        mStream->close();
        mStream.reset();
    }
}

void AudioEngine::playSnare() {
    if (!mSnareSamples.empty()) {
        mSnareFramePlayhead.store(0, std::memory_order_release);
        LOGI("Snare triggered");
    }
}

// Minimal WAV parser for PCM 16-bit mono OR stereo
static bool parseWav16(
        const uint8_t* data,
        size_t size,
        std::vector<int16_t>& outSamples,
        uint16_t& outChannels,
        uint32_t& outSampleRate) {

    if (size < 44) return false;

    // "RIFF"
    if (std::memcmp(data, "RIFF", 4) != 0) return false;
    // "WAVE"
    if (std::memcmp(data + 8, "WAVE", 4) != 0) return false;

    size_t offset = 12;
    uint16_t audioFormat = 0;
    uint16_t numChannels = 0;
    uint32_t sampleRate = 0;
    uint16_t bitsPerSample = 0;
    const uint8_t* dataChunkPtr = nullptr;
    uint32_t dataChunkSize = 0;

    while (offset + 8 <= size) {
        const char* chunkId = reinterpret_cast<const char*>(data + offset);
        uint32_t chunkSize = *reinterpret_cast<const uint32_t*>(data + offset + 4);
        size_t nextChunk = offset + 8 + chunkSize;
        if (nextChunk > size) break;

        if (std::memcmp(chunkId, "fmt ", 4) == 0) {
            if (chunkSize < 16) return false;

            audioFormat   = *reinterpret_cast<const uint16_t*>(data + offset + 8);
            numChannels   = *reinterpret_cast<const uint16_t*>(data + offset + 10);
            sampleRate    = *reinterpret_cast<const uint32_t*>(data + offset + 12);
            bitsPerSample = *reinterpret_cast<const uint16_t*>(data + offset + 22);

        } else if (std::memcmp(chunkId, "data", 4) == 0) {
            dataChunkPtr  = data + offset + 8;
            dataChunkSize = chunkSize;
        }

        offset = nextChunk;
    }

    if (!dataChunkPtr || dataChunkSize == 0) {
        LOGE("No 'data' chunk found in WAV");
        return false;
    }

    if (audioFormat != 1) { // PCM
        LOGE("WAV is not PCM (audioFormat=%u)", audioFormat);
        return false;
    }
    if (bitsPerSample != 16) {
        LOGE("WAV is not 16-bit (bitsPerSample=%u)", bitsPerSample);
        return false;
    }
    if (numChannels != 1 && numChannels != 2) {
        LOGE("WAV has unsupported channel count: %u", numChannels);
        return false;
    }

    size_t sampleCount = dataChunkSize / sizeof(int16_t);
    outSamples.resize(sampleCount);
    std::memcpy(outSamples.data(), dataChunkPtr, dataChunkSize);

    outChannels    = numChannels;
    outSampleRate  = sampleRate;

    LOGI("Loaded WAV: %zu samples, channels=%u, sampleRate=%u",
         sampleCount, numChannels, sampleRate);
    return true;
}
bool AudioEngine::loadSnareFromAsset(AAssetManager* mgr, const char* path) {
    AAsset* asset = AAssetManager_open(mgr, path, AASSET_MODE_BUFFER);
    if (!asset) {
        LOGE("AAssetManager_open failed for %s", path);
        return false;
    }

    size_t len = AAsset_getLength(asset);
    const uint8_t* buf = static_cast<const uint8_t*>(AAsset_getBuffer(asset));
    if (!buf || len == 0) {
        LOGE("Asset buffer invalid for %s", path);
        AAsset_close(asset);
        return false;
    }

    std::vector<int16_t> samples;
    uint16_t channels = 0;
    uint32_t sampleRate = 0;

    bool ok = parseWav16(buf, len, samples, channels, sampleRate);
    AAsset_close(asset);

    if (!ok) {
        LOGE("parseWav16 failed for %s", path);
        return false;
    }

    mSnareSamples   = std::move(samples);
    mSnareChannels  = channels;
    mSnareSampleRate = sampleRate;

    LOGI("Snare loaded: %zu samples, channels=%u, sampleRate=%u",
         mSnareSamples.size(), mSnareChannels, mSnareSampleRate);
    return true;
}
oboe::DataCallbackResult AudioEngine::onAudioReady(
        oboe::AudioStream* audioStream,
        void* audioData,
        int32_t numFrames) {

    float* out = static_cast<float*>(audioData);
    const int32_t outChannels = audioStream->getChannelCount();

    const uint16_t srcChannels = mSnareChannels;
    if (srcChannels == 0) {
        // No valid sample loaded
        std::memset(out, 0, sizeof(float) * numFrames * outChannels);
        return oboe::DataCallbackResult::Continue;
    }

    int32_t framePlayhead = mSnareFramePlayhead.load(std::memory_order_acquire);

    for (int32_t frame = 0; frame < numFrames; ++frame) {
        float left  = 0.0f;
        float right = 0.0f;

        if (framePlayhead >= 0) {
            // Compute base index into interleaved int16 array
            size_t baseIndex = static_cast<size_t>(framePlayhead) * srcChannels;
            size_t maxSamples = mSnareSamples.size();

            if (baseIndex + (srcChannels - 1) < maxSamples) {
                if (srcChannels == 1) {
                    float s = static_cast<float>(mSnareSamples[baseIndex]) / 32768.0f;
                    left = right = s;
                } else { // stereo
                    float sL = static_cast<float>(mSnareSamples[baseIndex + 0]) / 32768.0f;
                    float sR = static_cast<float>(mSnareSamples[baseIndex + 1]) / 32768.0f;
                    left  = sL;
                    right = sR;
                }

                // Advance to next frame in sample domain
                framePlayhead++;
            } else {
                // Reached end of sample
                framePlayhead = -1;
            }
        }

        // Write to output (downmix/upmix if needed)
        if (outChannels == 1) {
            out[frame] = 0.5f * (left + right);
        } else {
            out[frame * outChannels + 0] = left;
            if (outChannels >= 2) {
                out[frame * outChannels + 1] = right;
            }
        }
    }

    // Store updated playhead once per callback
    mSnareFramePlayhead.store(framePlayhead, std::memory_order_release);

    return oboe::DataCallbackResult::Continue;
}

void AudioEngine::onErrorAfterClose(
        oboe::AudioStream* stream,
        oboe::Result error) {
    LOGE("Stream error after close: %s", oboe::convertToText(error));
}