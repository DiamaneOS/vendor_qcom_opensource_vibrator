/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "vendor.qti.vibrator"

#include <cutils/properties.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <dirent.h>
#include <inttypes.h>
#include <linux/input.h>
#include <log/log.h>
#include <string.h>
#include <unistd.h>
#include <bits/epoll_event.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <sys/poll.h>
#include <thread>

#include "include/Vibrator.h"
#ifdef USE_EFFECT_STREAM
#include "effect.h"
#endif

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

#define STRONG_MAGNITUDE        0x7fff
#define MEDIUM_MAGNITUDE        0x5fff
#define LIGHT_MAGNITUDE         0x3fff
#define INVALID_VALUE           -1
#define CUSTOM_DATA_LEN         3
#define NAME_BUF_SIZE           32
#define PRIMITIVE_ID_MASK       0x8000
#define MAX_PATTERN_ID          32767
#define VIB_LED  0x01
#define VIB_AW   0x02

#define VIB_ALL (VIB_LED|VIB_AW)

#define MSM_CPU_LAHAINA         415
#define APQ_CPU_LAHAINA         439
#define MSM_CPU_SHIMA           450
#define MSM_CPU_SM8325          501
#define APQ_CPU_SM8325P         502
#define MSM_CPU_YUPIK           475
#define MSM_CPU_CAPE            530
#define APQ_CPU_CAPE            531
#define MSM_CPU_TARO            457
#define MSM_CPU_TARO_LTE        552
#define MSM_CPU_KALAMA          519
#define MSM_CPU_PINEAPPLE       557

#define test_bit(bit, array)    ((array)[(bit)/8] & (1<<((bit)%8)))

static const char LED_DEVICE[] = "/sys/class/leds/vibrator";
static const char HAPTICS_SYSFS[] = "/sys/class/qcom-haptics";
static const char AW_DEVICE[] = "/sys/class/leds/aw_vibrator";

static constexpr int32_t ComposeDelayMaxMs = 1000;
static constexpr int32_t ComposeSizeMax = 256;

/*
 * Awinic haptics (aw_vibrator, e.g. FP6). Effects play the fixed waveforms in
 * the chip RAM (/vendor/firmware/haptic_ram.bin, 24 kHz, f0 ~234 Hz); RTP
 * (streamed waveforms) is not used.
 *   wave 1: 29.8 ms (~7 cycles)    wave 2: 22.2 ms (~5 cycles)
 *   wave 3: 14.5 ms (~3.5 cycles)  wave 4:  4.3 ms (one cycle, looped by on())
 * Gain is the chip's digital gain, 0x01..AW_GAIN_MAX (0x80 = full scale).
 */
#define AW_RAM_WAVE_NUM         4
#define AW_LOOP_WAVE            4
#define AW_GAIN_MAX             0x80
/* The driver starts a waveform from its work queue, a few ms after the trigger */
#define AW_TRIGGER_MARGIN_MS    5
/*
 * on() up to this long plays one RAM waveform, longer ones loop AW_LOOP_WAVE.
 * Any positive on() length is accepted, as before: the driver timer takes an
 * int of ms and the HAL stops the loop at the end as well.
 */
#define AW_SHORT_ON_MAX_MS      50

/* Waveform lengths in ms, rounded up; index 0 = no waveform */
static constexpr int32_t kAwWaveMs[AW_RAM_WAVE_NUM + 1] = { 0, 30, 23, 15, 5 };

/*
 * Tuning table, set by feel test on the FP6 (2026-09-26): wave 3 is
 * the tick (light 0x18 for slider steps, medium 0x40 for the keyboard); waves 2
 * and 1 feel alike and wave 1 at full gain feels cheap, so clicks, heavy clicks
 * and thuds use wave 2 and wave 1 is not used for taps.
 * Adjust by feel: the RAM waveform each effect plays and its gain
 * for EffectStrength LIGHT, MEDIUM and STRONG (touch feedback follows the
 * user's vibration intensity setting; MEDIUM is the default). repeatGapMs > 0
 * plays the waveform again after that much silence.
 * Composition primitives use the second table: gain = scale * fullGain.
 */
struct AwEffect {
    Effect effect;
    uint8_t wave;
    uint8_t gain[3];
    int32_t repeatGapMs;
};

static constexpr AwEffect kAwEffects[] = {
    /* effect                wave   LIGHT MEDIUM STRONG   repeatGapMs */
    { Effect::TICK,            3, { 0x18, 0x40, 0x80 },   0 },
    { Effect::TEXTURE_TICK,    3, { 0x18, 0x28, 0x38 },   0 },
    { Effect::CLICK,           2, { 0x20, 0x30, 0x60 },   0 },
    { Effect::POP,             2, { 0x20, 0x30, 0x48 },   0 },
    { Effect::HEAVY_CLICK,     2, { 0x50, 0x68, 0x80 },   0 },
    { Effect::THUD,            2, { 0x40, 0x58, 0x70 },   0 },
    { Effect::DOUBLE_CLICK,    2, { 0x20, 0x30, 0x60 }, 100 },
};

struct AwPrimitive {
    CompositePrimitive primitive;
    uint8_t wave;
    uint8_t fullGain;
};

static constexpr AwPrimitive kAwPrimitives[] = {
    /* primitive                       wave  fullGain */
    { CompositePrimitive::NOOP,          0,  0x00 },
    /* The keyboard composes CLICK at config_keyboardHapticFeedbackFixedAmplitude
       (0.5 in the FP6 overlay: 0x40, the medium tick); SystemUI slider steps
       compose PRIMITIVE_TICK at 0.5 (0x18, the light tick). */
    { CompositePrimitive::CLICK,         3,  0x80 },
    { CompositePrimitive::LIGHT_TICK,    3,  0x30 },    /* Java PRIMITIVE_TICK */
    { CompositePrimitive::LOW_TICK,      3,  0x28 },
};

static constexpr bool awTablesValid() {
    for (const AwEffect& e : kAwEffects) {
        if (e.wave < 1 || e.wave > AW_RAM_WAVE_NUM ||
            e.repeatGapMs < 0 || e.repeatGapMs > ComposeDelayMaxMs)
            return false;
        for (uint8_t g : e.gain)
            if (g < 1 || g > AW_GAIN_MAX)
                return false;
    }
    for (const AwPrimitive& p : kAwPrimitives) {
        if (p.wave > AW_RAM_WAVE_NUM || p.fullGain > AW_GAIN_MAX ||
            (p.wave != 0 && p.fullGain < 1))
            return false;
    }
    return true;
}
static_assert(awTablesValid(), "Awinic tuning table out of range");

static const AwEffect *awFindEffect(Effect effect) {
    for (const AwEffect& e : kAwEffects)
        if (e.effect == effect)
            return &e;
    return nullptr;
}

static const AwPrimitive *awFindPrimitive(CompositePrimitive primitive) {
    for (const AwPrimitive& p : kAwPrimitives)
        if (p.primitive == primitive)
            return &p;
    return nullptr;
}

/* Time a RAM waveform takes from the trigger until it has played out */
static int32_t awWavePlayMs(uint8_t wave) {
    if (wave < 1 || wave > AW_RAM_WAVE_NUM)
        return 0;
    return kAwWaveMs[wave] + AW_TRIGGER_MARGIN_MS;
}

enum composeEvent {
    STOP_COMPOSE = 0,
};

InputFFDevice::InputFFDevice()
{
    DIR *dp;
    FILE *fp = NULL;
    struct dirent *dir;
    uint8_t ffBitmask[FF_CNT / 8];
    char devicename[PATH_MAX];
    const char *INPUT_DIR = "/dev/input/";
    char name[NAME_BUF_SIZE];
    int fd, ret;
    int soc = property_get_int32("ro.vendor.qti.soc_id", -1);

    mVibraFd = INVALID_VALUE;
    mSupportGain = false;
    mSupportEffects = false;
    mSupportExternalControl = false;
    mCurrAppId = INVALID_VALUE;
    mCurrMagnitude = 0x7fff;
    mInExternalControl = false;

    /*
     * An Awinic vibrator (FP6) is driven through its sysfs nodes by
     * LedVibratorDevice, and the input devices are never used. Do not open
     * them: the probe below opens every one read-write, touchscreen and keys
     * included, so the service needs neither the input group nor
     * input_device access.
     */
    snprintf(devicename, sizeof(devicename), "%s/%s", AW_DEVICE, "activate");
    if (access(devicename, F_OK) == 0) {
        ALOGD("Awinic vibrator present, input devices not probed");
        return;
    }

    dp = opendir(INPUT_DIR);
    if (!dp) {
        ALOGE("open %s failed, errno = %d", INPUT_DIR, errno);
        return;
    }

    memset(ffBitmask, 0, sizeof(ffBitmask));
    while ((dir = readdir(dp)) != NULL){
        if (dir->d_name[0] == '.' &&
            (dir->d_name[1] == '\0' ||
             (dir->d_name[1] == '.' && dir->d_name[2] == '\0')))
            continue;

        snprintf(devicename, PATH_MAX, "%s%s", INPUT_DIR, dir->d_name);
        fd = TEMP_FAILURE_RETRY(open(devicename, O_RDWR));
        if (fd < 0) {
            ALOGE("open %s failed, errno = %d", devicename, errno);
            continue;
        }

        ret = TEMP_FAILURE_RETRY(ioctl(fd, EVIOCGNAME(sizeof(name)), name));
        if (ret == -1) {
            ALOGE("get input device name %s failed, errno = %d\n", devicename, errno);
            close(fd);
            continue;
        }

        if (strcmp(name, "qcom-hv-haptics") && strcmp(name, "qti-haptics")) {
            ALOGD("not a qcom/qti haptics device\n");
            close(fd);
            continue;
        }

        ALOGI("%s is detected at %s\n", name, devicename);
        ret = TEMP_FAILURE_RETRY(ioctl(fd, EVIOCGBIT(EV_FF, sizeof(ffBitmask)), ffBitmask));
        if (ret == -1) {
            ALOGE("ioctl failed, errno = %d", errno);
            close(fd);
            continue;
        }

        if (test_bit(FF_CONSTANT, ffBitmask) ||
                test_bit(FF_PERIODIC, ffBitmask)) {
            mVibraFd = fd;
            if (test_bit(FF_CUSTOM, ffBitmask))
                mSupportEffects = true;
            if (test_bit(FF_GAIN, ffBitmask))
                mSupportGain = true;

            if (soc <= 0 && (fp = fopen("/sys/devices/soc0/soc_id", "r")) != NULL) {
                fscanf(fp, "%u", &soc);
                fclose(fp);
            }
            switch (soc) {
            case MSM_CPU_LAHAINA:
            case APQ_CPU_LAHAINA:
            case MSM_CPU_SHIMA:
            case MSM_CPU_SM8325:
            case APQ_CPU_SM8325P:
            case MSM_CPU_YUPIK:
                mSupportExternalControl = true;
                break;
            default:
                mSupportExternalControl = false;
                break;
            }
            break;
        }

        close(fd);
    }

    closedir(dp);
}

/** Play vibration
 *
 *  @param effectId:  ID of the predefined effect will be played. If effectId is valid
 *                    (non-negative value), the timeoutMs value will be ignored, and the
 *                    real playing length will be set in param@playLengtMs and returned
 *                    to VibratorService. If effectId is invalid, value in param@timeoutMs
 *                    will be used as the play length for playing a constant effect.
 *  @param timeoutMs: playing length, non-zero means playing, zero means stop playing.
 *  @param playLengthMs: the playing length in ms unit which will be returned to
 *                    VibratorService if the request is playing a predefined effect.
 *                    The custom_data in periodic is reused for returning the playLengthMs
 *                    from kernel space to userspace if the pattern is defined in kernel
 *                    driver. It's been defined with following format:
 *                       <effect-ID, play-time-in-seconds, play-time-in-milliseconds>.
 *                    The effect-ID is used for passing down the predefined effect to
 *                    kernel driver, and the rest two parameters are used for returning
 *                    back the real playing length from kernel driver.
 */
int InputFFDevice::play(int effectId, uint32_t timeoutMs, long *playLengthMs) {
    struct ff_effect effect;
    struct input_event play;
    int16_t data[CUSTOM_DATA_LEN] = {0, 0, 0};
    int ret;
#ifdef USE_EFFECT_STREAM
    const struct effect_stream *stream;
#endif

    mtx.lock();
    /* For QMAA compliance, return OK even if vibrator device doesn't exist */
    if (mVibraFd == INVALID_VALUE) {
        if (playLengthMs != NULL)
            *playLengthMs = 0;
            mtx.unlock();
            return 0;
    }

    if (timeoutMs != 0) {
        if (mCurrAppId != INVALID_VALUE) {
            ret = TEMP_FAILURE_RETRY(ioctl(mVibraFd, EVIOCRMFF, mCurrAppId));
            if (ret == -1) {
                ALOGE("ioctl EVIOCRMFF failed, errno = %d", -errno);
                goto errout;
            }
            mCurrAppId = INVALID_VALUE;
        }

        memset(&effect, 0, sizeof(effect));
        if (effectId != INVALID_VALUE) {
            data[0] = effectId;
            effect.type = FF_PERIODIC;
            effect.u.periodic.waveform = FF_CUSTOM;
            effect.u.periodic.magnitude = mCurrMagnitude;
            effect.u.periodic.custom_data = data;
            effect.u.periodic.custom_len = sizeof(int16_t) * CUSTOM_DATA_LEN;
#ifdef USE_EFFECT_STREAM
            stream = get_effect_stream(effectId);
            if (stream != NULL) {
                effect.u.periodic.custom_data = (int16_t *)stream;
                effect.u.periodic.custom_len = sizeof(*stream);
            }
#endif
        } else {
            effect.type = FF_CONSTANT;
            effect.u.constant.level = mCurrMagnitude;
            effect.replay.length = timeoutMs;
        }

        effect.id = mCurrAppId;
        effect.replay.delay = 0;

        ret = TEMP_FAILURE_RETRY(ioctl(mVibraFd, EVIOCSFF, &effect));
        if (ret == -1) {
            ALOGE("ioctl EVIOCSFF failed, errno = %d", -errno);
            goto errout;
        }

        mCurrAppId = effect.id;
        if (effectId != INVALID_VALUE && playLengthMs != NULL) {
            *playLengthMs = data[1] * 1000 + data[2];
#ifdef USE_EFFECT_STREAM
            if (stream != NULL && stream->play_rate_hz != 0)
                *playLengthMs = ((stream->length * 1000) / stream->play_rate_hz) + 1;
#endif
        }

        play.value = 1;
        play.type = EV_FF;
        play.code = mCurrAppId;
        play.time.tv_sec = 0;
        play.time.tv_usec = 0;
        ret = TEMP_FAILURE_RETRY(write(mVibraFd, (const void*)&play, sizeof(play)));
        if (ret == -1) {
            ALOGE("write failed, errno = %d\n", -errno);
            ret = TEMP_FAILURE_RETRY(ioctl(mVibraFd, EVIOCRMFF, mCurrAppId));
            if (ret == -1)
                ALOGE("ioctl EVIOCRMFF failed, errno = %d", -errno);
            goto errout;
        }
    } else if (mCurrAppId != INVALID_VALUE) {
        ret = TEMP_FAILURE_RETRY(ioctl(mVibraFd, EVIOCRMFF, mCurrAppId));
        if (ret == -1) {
            ALOGE("ioctl EVIOCRMFF failed, errno = %d", -errno);
            goto errout;
        }
        mCurrAppId = INVALID_VALUE;
    }
    mtx.unlock();
    return 0;

errout:
    mCurrAppId = INVALID_VALUE;
    mtx.unlock();
    return ret;
}

int InputFFDevice::on(int32_t timeoutMs) {
    return play(INVALID_VALUE, timeoutMs, NULL);
}

int InputFFDevice::off() {
    return play(INVALID_VALUE, 0, NULL);
}

int InputFFDevice::setAmplitude(uint8_t amplitude) {
    int tmp, ret;
    struct input_event ie;
    /* For QMAA compliance, return OK even if vibrator device doesn't exist */
    if (mVibraFd == INVALID_VALUE)
        return 0;

    tmp = amplitude * (STRONG_MAGNITUDE - LIGHT_MAGNITUDE) / 255;
    tmp += LIGHT_MAGNITUDE;
    ie.type = EV_FF;
    ie.code = FF_GAIN;
    ie.value = tmp;

    ret = TEMP_FAILURE_RETRY(write(mVibraFd, &ie, sizeof(ie)));
    if (ret == -1) {
        ALOGE("write FF_GAIN failed, errno = %d", -errno);
        return ret;
    }

    mCurrMagnitude = tmp;
    return 0;
}

int InputFFDevice::playEffect(int effectId, EffectStrength es, long *playLengthMs) {
    if (effectId > MAX_PATTERN_ID) {
        ALOGE("effect id %d exceeds %d", effectId, MAX_PATTERN_ID);
        return -1;
    }

    switch (es) {
    case EffectStrength::LIGHT:
        mCurrMagnitude = LIGHT_MAGNITUDE;
        break;
    case EffectStrength::MEDIUM:
        mCurrMagnitude = MEDIUM_MAGNITUDE;
        break;
    case EffectStrength::STRONG:
        mCurrMagnitude = STRONG_MAGNITUDE;
        break;
    default:
        return -1;
    }

    return play(effectId, INVALID_VALUE, playLengthMs);
}

int InputFFDevice::playPrimitive(int primitiveId, float amplitude, long *playLengthMs) {
    int8_t tmp;
    int ret = 0;

    if (primitiveId > MAX_PATTERN_ID) {
        ALOGE("primitive id %d exceeds %d", primitiveId, MAX_PATTERN_ID);
        return -1;
    }

    primitiveId |= PRIMITIVE_ID_MASK;
    tmp = (uint8_t)(amplitude * 0xff);
    mCurrMagnitude = tmp * (STRONG_MAGNITUDE - LIGHT_MAGNITUDE) / 255;
    mCurrMagnitude += LIGHT_MAGNITUDE;

    ret = play(primitiveId, INVALID_VALUE, playLengthMs);
    if (ret != 0)
        ALOGE("Failed to play primitive %d", primitiveId);

    return ret;
}

LedVibratorDevice::LedVibratorDevice() {
    char devicename[PATH_MAX];
    int fd;

    mDetected = false;
    vibrator_dev = VIB_ALL;

    snprintf(devicename, sizeof(devicename), "%s/%s", LED_DEVICE, "activate");
    fd = TEMP_FAILURE_RETRY(open(devicename, O_RDWR));
    if (fd < 0) {
        ALOGE("vibrator open %s failed, errno = %d", devicename, errno);
        vibrator_dev &= ~ VIB_LED;
 //       return;
    } else {
        close(fd);
    }

   snprintf(devicename, sizeof(devicename), "%s/%s", AW_DEVICE, "activate");
    fd = TEMP_FAILURE_RETRY(open(devicename, O_RDWR));
    if (fd < 0) {
        ALOGE("vibrator open %s failed, errno = %d", devicename, errno);
        vibrator_dev &= ~ VIB_AW;
    } else {
        close(fd);
    }
    ALOGE("vibrator device = %d", vibrator_dev);
    if(!vibrator_dev)
        return;
    ALOGE("vibrator true");
    mDetected = true;

    /* A restarted service must not leave a vibration from its previous instance running */
    if (vibrator_dev & VIB_AW)
        off();
}

int LedVibratorDevice::write_value(const char *file, const char *value) {
    int fd;
    int ret;

    fd = TEMP_FAILURE_RETRY(open(file, O_WRONLY));
    if (fd < 0) {
        ALOGE("open %s failed, errno = %d", file, errno);
        return -errno;
    }

    ret = TEMP_FAILURE_RETRY(write(fd, value, strlen(value) + 1));
    if (ret == -1) {
        ret = -errno;
    } else if (ret != strlen(value) + 1) {
        /* even though EAGAIN is an errno value that could be set
           by write() in some cases, none of them apply here.  So, this return
           value can be clearly identified when debugging and suggests the
           caller that it may try to call vibrator_on() again */
        ret = -EAGAIN;
    } else {
        ret = 0;
    }

    errno = 0;
    close(fd);

    return ret;
}

int LedVibratorDevice::aw_write(const char *node, const char *value) {
    char file[PATH_MAX];

    snprintf(file, sizeof(file), "%s/%s", AW_DEVICE, node);
    return write_value(file, value);
}

int LedVibratorDevice::awSetGain(uint8_t gain) {
    char value[8];

    if (!(vibrator_dev & VIB_AW))
        return -ENODEV;

    if (gain > AW_GAIN_MAX)
        gain = AW_GAIN_MAX;
    snprintf(value, sizeof(value), "0x%02x", gain);
    return aw_write("gain", value);
}

/*
 * Play one RAM waveform once. Writing "brightness" puts the driver in RAM mode
 * (boost on) and plays sequencer slots from slot 0 until an empty slot; the
 * chip stops by itself at the end. Every slot used is rewritten here, so an
 * infinite loop count left in slot 0 by awPlayLoop() cannot carry over (RAM
 * mode has no driver timer).
 */
int LedVibratorDevice::awPlayRam(uint8_t wave, uint8_t gain) {
    char value[16];
    int ret;

    if (!(vibrator_dev & VIB_AW))
        return -ENODEV;
    if (wave < 1 || wave > AW_RAM_WAVE_NUM)
        return -EINVAL;

    ret = awSetGain(gain);
    if (ret < 0)
        goto error;

    snprintf(value, sizeof(value), "0x00 0x%02x", wave);
    ret = aw_write("seq", value);
    if (ret < 0)
        goto error;

    ret = aw_write("loop", "0x00 0x00");
    if (ret < 0)
        goto error;

    /* end of sequence */
    ret = aw_write("seq", "0x01 0x00");
    if (ret < 0)
        goto error;

    ret = aw_write("brightness", "1");
    if (ret < 0)
        goto error;

    return 0;

error:
    ALOGE("AwVibrator failed to play wave %d, ret: %d", wave, ret);
    return ret;
}

/*
 * Loop the one-cycle waveform for timeoutMs. Writing "activate" uses the
 * device tree play mode, RAM loop (mode = 5 on FP6), in which the driver arms
 * a timer for "duration" and stops the loop when it fires. The caller stops
 * the motor at the end as well.
 */
int LedVibratorDevice::awPlayLoop(int32_t timeoutMs, uint8_t gain) {
    char value[16];
    int ret;

    if (!(vibrator_dev & VIB_AW))
        return -ENODEV;
    if (timeoutMs <= 0)
        return -EINVAL;

    ret = awSetGain(gain);
    if (ret < 0)
        goto error;

    snprintf(value, sizeof(value), "0x00 0x%02x", AW_LOOP_WAVE);
    ret = aw_write("seq", value);
    if (ret < 0)
        goto error;

    /* 0x0f: repeat until stopped */
    ret = aw_write("loop", "0x00 0x0f");
    if (ret < 0)
        goto error;

    ret = aw_write("seq", "0x01 0x00");
    if (ret < 0)
        goto error;

    snprintf(value, sizeof(value), "%d\n", timeoutMs);
    ret = aw_write("duration", value);
    if (ret < 0)
        goto error;

    ret = aw_write("activate", "1");
    if (ret < 0)
        goto error;

    return 0;

error:
    ALOGE("AwVibrator failed to loop for %d ms, ret: %d", timeoutMs, ret);
    return ret;
}

int LedVibratorDevice::on(int32_t timeoutMs) {
    char file[PATH_MAX];
    char value[32];
    int ret;

    /* Awinic playback goes through Vibrator::awOn() */
    if (vibrator_dev & VIB_AW)
        return -EINVAL;

    snprintf(file, sizeof(file), "%s/%s", LED_DEVICE, "state");
    ret = write_value(file, "1");
    if (ret < 0)
       goto error;

    snprintf(file, sizeof(file), "%s/%s", LED_DEVICE, "duration");
    snprintf(value, sizeof(value), "%u\n", timeoutMs);
    ret = write_value(file, value);
    if (ret < 0)
       goto error;

    snprintf(file, sizeof(file), "%s/%s", LED_DEVICE, "activate");
    ret = write_value(file, "1");
    if (ret < 0)
       goto error;

    return 0;

error:
    ALOGE("Failed to turn on vibrator ret: %d\n", ret);
    return ret;
}

int LedVibratorDevice::off()
{
    char file[PATH_MAX];
    int ret, ret2;
    ALOGD("LedVibrator device = %d ",vibrator_dev);
    if(vibrator_dev & VIB_AW)
    {
        /*
         * Either write sets the driver state to 0, and its work then cancels
         * the loop timer and stops the chip. Both are written so that one
         * failing still stops the motor. The sequencer is not touched (the
         * old "index" write here left slot 0 looping forever); awPlayRam()
         * rewrites every slot it plays before triggering.
         */
        ret = aw_write("activate", "0");
        ret2 = aw_write("brightness", "0");
        if (ret < 0 && ret2 < 0) {
            ALOGE("AwVibrator failed to stop, ret: %d %d", ret, ret2);
            return ret;
        }

        return 0;
    }

    snprintf(file, sizeof(file), "%s/%s", LED_DEVICE, "activate");
    ret = write_value(file, "0");
    return ret;
}

Vibrator::Vibrator() {
    struct epoll_event ev;

    epollfd = INVALID_VALUE;
    pipefd[0] = INVALID_VALUE;
    pipefd[1] = INVALID_VALUE;
    inComposition = false;

    mAwJob = AwJob();
    mAwJobQueued = false;
    mAwGeneration = 0;
    mAwOnActive = false;
    mAwStopped = true;      /* LedVibratorDevice() stopped it */
    mAwAmplitudeGain = AW_GAIN_MAX;
    mAwExit = false;
    if (isAw())
        mAwThread = std::thread(awPlayThread, this);

    if (!ff.mSupportEffects)
        return;

    if (pipe(pipefd)) {
        ALOGE("Failed to get pipefd error=%d", errno);
        return;
    }

    epollfd = epoll_create1(0);
    if (epollfd < 0) {
        ALOGE("Failed to create epoll fd error=%d", errno);
        goto pipefd_close;
    }

    ev.events = EPOLLIN;
    ev.data.fd = pipefd[0];

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, pipefd[0], &ev) == -1) {
        ALOGE("Failed to add pipefd to epoll ctl error=%d", errno);
        goto epollfd_close;
    }

    return;

epollfd_close:
    close(epollfd);
    epollfd = INVALID_VALUE;
pipefd_close:
    close(pipefd[0]);
    close(pipefd[1]);
    pipefd[0] = INVALID_VALUE;
    pipefd[1] = INVALID_VALUE;
}

Vibrator::~Vibrator() {
    if (mAwThread.joinable()) {
        {
            std::lock_guard<std::mutex> lock(mAwLock);
            mAwExit = true;
        }
        mAwCv.notify_all();
        mAwThread.join();
    }
    if (epollfd != INVALID_VALUE)
        close(epollfd);
    if (pipefd[0] != INVALID_VALUE)
        close(pipefd[0]);
    if (pipefd[1] != INVALID_VALUE)
        close(pipefd[1]);
}

ndk::ScopedAStatus Vibrator::getCapabilities(int32_t* _aidl_return) {
    *_aidl_return = IVibrator::CAP_ON_CALLBACK;

    if (isAw()) {
        *_aidl_return |= IVibrator::CAP_PERFORM_CALLBACK | IVibrator::CAP_AMPLITUDE_CONTROL |
                         IVibrator::CAP_COMPOSE_EFFECTS;
        ALOGD("QTI Vibrator reporting capabilities: %d", *_aidl_return);
        return ndk::ScopedAStatus::ok();
    }

    if (ledVib.mDetected) {
        *_aidl_return |= IVibrator::CAP_PERFORM_CALLBACK;
        ALOGD("QTI Vibrator reporting capabilities: %d", *_aidl_return);
        return ndk::ScopedAStatus::ok();
    }

    if (ff.mSupportGain)
        *_aidl_return |= IVibrator::CAP_AMPLITUDE_CONTROL;
    if (ff.mSupportEffects) {
       *_aidl_return |= IVibrator::CAP_PERFORM_CALLBACK;
       *_aidl_return |= IVibrator::CAP_COMPOSE_EFFECTS;
    }
    if (ff.mSupportExternalControl)
        *_aidl_return |= IVibrator::CAP_EXTERNAL_CONTROL;

    ALOGD("QTI Vibrator reporting capabilities: %d", *_aidl_return);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::off() {
    int ret;
    int composeEven = STOP_COMPOSE;

    ALOGD("QTI Vibrator off");
    if (isAw())
        return awOff();

    if (ledVib.mDetected)
        ret = ledVib.off();
    else
        ret = ff.off();
    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));

    if (inComposition) {
        ret = write(pipefd[1], &composeEven, sizeof(composeEven));
        if (ret < 0) {
            ALOGE("Failed to send STOP_COMPOSE event");
            return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
        }
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::on(int32_t timeoutMs,
                                const std::shared_ptr<IVibratorCallback>& callback) {
    int ret;

    ALOGD("Vibrator on for timeoutMs: %d", timeoutMs);
    if (isAw())
        return awOn(timeoutMs, callback);

    if (ledVib.mDetected)
        ret = ledVib.on(timeoutMs);
    else
        ret = ff.on(timeoutMs);

    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));

    if (callback != nullptr) {
        std::thread([=] {
            ALOGD("Starting on on another thread");
            usleep(timeoutMs * 1000);
            ALOGD("Notifying on complete");
            if (!callback->onComplete().isOk()) {
                ALOGE("Failed to call onComplete");
            }
        }).detach();
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::perform(Effect effect, EffectStrength es, const std::shared_ptr<IVibratorCallback>& callback, int32_t* _aidl_return) {
    long playLengthMs;
    int ret;

    ALOGD("Vibrator perform EffectStrength %d", es);
    if (isAw())
        return awPerform(effect, es, callback, _aidl_return);

    if (ledVib.mDetected)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    ALOGD("Vibrator perform effect %d", effect);
    if (Offload.mEnabled == 1) {
        if ((effect < Effect::CLICK) ||
            ((effect > Effect::HEAVY_CLICK) && (effect < Effect::RINGTONE_12)) ||
            (effect > Effect::RINGTONE_15))
            return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }
    else {
        if (effect < Effect::CLICK ||  effect > Effect::HEAVY_CLICK)
            return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    if (es != EffectStrength::LIGHT && es != EffectStrength::MEDIUM && es != EffectStrength::STRONG)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    ret = ff.playEffect((static_cast<int>(effect)), es, &playLengthMs);
    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));

    if (callback != nullptr) {
        std::thread([=] {
            ALOGD("Starting perform on another thread");
            usleep(playLengthMs * 1000);
            ALOGD("Notifying perform complete");
            callback->onComplete();
        }).detach();
    }

    *_aidl_return = playLengthMs;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedEffects(std::vector<Effect>* _aidl_return) {
    if (isAw()) {
        _aidl_return->clear();
        for (const AwEffect& e : kAwEffects)
            _aidl_return->push_back(e.effect);
        return ndk::ScopedAStatus::ok();
    }

    if (ledVib.mDetected)
        return ndk::ScopedAStatus::ok();

    if (Offload.mEnabled == 1)
        *_aidl_return = {Effect::CLICK, Effect::DOUBLE_CLICK, Effect::TICK, Effect::THUD,
                         Effect::POP, Effect::HEAVY_CLICK, Effect::RINGTONE_12,
                         Effect::RINGTONE_13, Effect::RINGTONE_14, Effect::RINGTONE_15};
    else
        *_aidl_return = {Effect::CLICK, Effect::DOUBLE_CLICK, Effect::TICK, Effect::THUD,
                         Effect::POP, Effect::HEAVY_CLICK};

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setAmplitude(float amplitude) {
    uint8_t tmp;
    int ret;

    if (isAw())
        return awSetAmplitude(amplitude);

    if (ledVib.mDetected)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    ALOGD("Vibrator set amplitude: %f", amplitude);

    if (amplitude <= 0.0f || amplitude > 1.0f)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));

    if (ff.mInExternalControl)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    tmp = (uint8_t)(amplitude * 0xff);
    ret = ff.setAmplitude(tmp);
    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setExternalControl(bool enabled) {
    if (ledVib.mDetected)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    ALOGD("Vibrator set external control: %d", enabled);
    if (!ff.mSupportExternalControl)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    ff.mInExternalControl = enabled;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getCompositionDelayMax(int32_t* maxDelayMs) {
    *maxDelayMs = ComposeDelayMaxMs;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getCompositionSizeMax(int32_t* maxSize) {
    *maxSize = ComposeSizeMax;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedPrimitives(std::vector<CompositePrimitive>* supported) {
    if (isAw()) {
        supported->clear();
        for (const AwPrimitive& p : kAwPrimitives)
            supported->push_back(p.primitive);
        return ndk::ScopedAStatus::ok();
    }

    *supported =  {
        CompositePrimitive::NOOP,   CompositePrimitive::CLICK,
        CompositePrimitive::THUD,   CompositePrimitive::SPIN,
        CompositePrimitive::QUICK_RISE, CompositePrimitive::SLOW_RISE,
        CompositePrimitive::QUICK_FALL, CompositePrimitive::LIGHT_TICK,
        CompositePrimitive::LOW_TICK,
    };
    return ndk::ScopedAStatus::ok();
}

static int getPrimitiveDurationFromSysfs(uint32_t primitive_id, int32_t* durationMs) {
    int count = 0;
    int fd = 0;
    int ret = 0;
    /* the Max primitive id is 32767, so define the size of primitive_buf to 6 */
    char primitive_buf[6];
    /* the max primitive_duration is the max value of int32, so define the size to 10 */
    char primitive_duration[10];
    char primitive_duration_sysfs[50];

    ret = snprintf(primitive_duration_sysfs, sizeof(primitive_duration_sysfs), "%s%s", HAPTICS_SYSFS, "/primitive_duration");
    if (ret < 0) {
        ALOGE("Failed to get primitive duration node, ret = %d\n", ret);
        return ret;
    }

    count = snprintf(primitive_buf, sizeof(primitive_buf), "%d%c", primitive_id, '\n');
    if (count < 0) {
        ALOGE("Failed to get primitive id, count = %d\n", count);
        ret = count;
        return ret;
    }

    fd = TEMP_FAILURE_RETRY(open(primitive_duration_sysfs, O_RDWR));
    if (fd < 0) {
        ALOGE("open %s failed, errno = %d", primitive_duration_sysfs, errno);
        ret = fd;
        return ret;
    }

    ret = TEMP_FAILURE_RETRY(write(fd, primitive_buf, count));
    if (ret < 0) {
        ALOGE("write primitive %d failed, errno = %d", primitive_id, errno);
        goto close_fd;
    }

    ret = TEMP_FAILURE_RETRY(lseek(fd, 0, SEEK_SET));
    if (ret < 0) {
        ALOGE("lseek fd to file head failed, errno = %d", errno);
        goto close_fd;
    }

    ret = TEMP_FAILURE_RETRY(read(fd, primitive_duration, sizeof(primitive_duration)));
    if (ret < 0) {
        ALOGE("read primitive %d failed, errno = %d", primitive_id, errno);
        goto close_fd;
    }

    *durationMs = atoi(primitive_duration);
    *durationMs /= 1000;

close_fd:
    ret = TEMP_FAILURE_RETRY(close(fd));
    if (ret < 0) {
        ALOGE("close primitive duration device failed, errno = %d", errno);
        return ret;
    }

    return ret;
}

ndk::ScopedAStatus Vibrator::getPrimitiveDuration(CompositePrimitive primitive,
                                                  int32_t* durationMs) {
    uint32_t primitive_id = static_cast<uint32_t>(primitive);
    int ret = 0;

    if (isAw()) {
        const AwPrimitive *p = awFindPrimitive(primitive);

        if (p == nullptr)
            return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
        *durationMs = awWavePlayMs(p->wave);
        ALOGD("primitive-%d duration is %dms", static_cast<int>(primitive), *durationMs);
        return ndk::ScopedAStatus::ok();
    }

#ifdef USE_EFFECT_STREAM
    primitive_id |= PRIMITIVE_ID_MASK ;
    const struct effect_stream *stream;
    stream = get_effect_stream(primitive_id);
    if (stream != NULL && stream->play_rate_hz != 0)
        *durationMs = ((stream->length * 1000) / stream->play_rate_hz) + 1;

    ALOGD("primitive-%d duration is %dms", primitive, *durationMs);
    return ndk::ScopedAStatus::ok();
#endif

    ret = getPrimitiveDurationFromSysfs(primitive_id, durationMs);
    if (ret < 0)
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);

    ALOGD("primitive-%d duration is %dms", primitive, *durationMs);

    return ndk::ScopedAStatus::ok();
}

void Vibrator::composePlayThread(Vibrator *vibrator,
                            const std::vector<CompositeEffect>& composite,
                            const std::shared_ptr<IVibratorCallback>& callback){
    struct epoll_event events;
    long playLengthMs = 0;
    int nfd = 0;
    int status = 0;
    int ret = 0;

    ALOGD("start a new thread for composeEffect");
    for (auto& e : composite) {
        if (e.delayMs) {
            nfd = epoll_wait(vibrator->epollfd, &events, 1, e.delayMs);
            if ((nfd == -1) && (errno != EINTR)) {
                ALOGE("Failed to wait delayMs, error=%d", errno);
                break;
            }

            if (nfd > 0) {
                /* It's supposed that STOP_COMPOSE command is received so quit the composition */
                ret = read(vibrator->pipefd[0], &status, sizeof(int));
                if (ret < 0) {
                    ALOGE("Failed to read stop status from pipe(delayMs), status = %d", status);
                    break;
                }
                if (status == STOP_COMPOSE)
                    break;
            }
        }

        vibrator->ff.playPrimitive((static_cast<int>(e.primitive)), e.scale, &playLengthMs);
        nfd = epoll_wait(vibrator->epollfd, &events, 1, playLengthMs);
        if (nfd == -1 && (errno != EINTR)) {
            ALOGE("Failed to wait sleep playLengthMs, error=%d", errno);
            break;
        }

        if (nfd > 0) {
            /* It's supposed that STOP_COMPOSE command is received so quit the composition */
            ret = read(vibrator->pipefd[0], &status, sizeof(int));
            if (ret < 0) {
                ALOGE("Failed to read stop status from pipe(playLengthMs), status = %d", status);
                break;
            }
            if (status == STOP_COMPOSE) {

                /*
                 * There is a corner case that the off() command could be executed in
                 * main thread before the primitive play is triggered in the child thread,
                 * such as, when playing a very short primitive effect while the system is
                 * pretty busy (one example is enabling all kernel console log after executed
                 * "echo Y > /sys/module/printk/parameters/ignore_loglevel"), the child thread
                 * may not be able to schedule out for running before the main thread times out
                 * on the primitive duration and sent the off() command, there won't be any
                 * off() command coming again to stop the primitive effect after it's triggered.
                 *
                 * However, the primitive could be played out and stopped automatically but the
                 * haptics driver does expect an explicit off() command to restore HW/SW logic
                 * after that, so call it here. It would result a redundant off() command in
                 * normal case but it won't do any harm because it would be ignored and not sent
                 * to haptics driver because of an invalid mCurrAppId. It would also result in the
                 * primitive effect to stop immediately right after it's triggered in such
                 * corner case. But considering the main thread has stopped it before off() is
                 * called here, take this as a limitation and it is expected not playing the
                 * vibration out.
                 */

                vibrator->ff.off();
                break;
            }
        }
    }

    ALOGD("Notifying composite complete, playlength= %ld", playLengthMs);
    if (callback)
        callback->onComplete();

    vibrator->inComposition = false;
}

ndk::ScopedAStatus Vibrator::compose(const std::vector<CompositeEffect>& composite,
                                     const std::shared_ptr<IVibratorCallback>& callback) {
    int status, nfd = 0, durationMs = 0, timeoutMs = 0;
    struct epoll_event events;

    if (isAw())
        return awCompose(composite, callback);

    if (composite.size() > ComposeSizeMax) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    std::vector<CompositePrimitive> supported;
    getSupportedPrimitives(&supported);

    for (auto& e : composite) {
        if (e.delayMs > ComposeDelayMaxMs) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
        if (e.scale < 0.0f || e.scale > 1.0f) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
        if (std::find(supported.begin(), supported.end(), e.primitive) == supported.end()) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
        }

        getPrimitiveDuration(e.primitive, &durationMs);
        timeoutMs += durationMs + e.delayMs;
    }

    /*
     * wait for 2 times of the play length timeout to make sure last play has been
     * terminated successfully.
     */
    timeoutMs = (timeoutMs + 10) * 2;
    /* Stop previous composition if it has not yet been completed */
    if (inComposition) {
        ALOGD("Last composePlayThread has not done yet, stop it manually");
        off();

        while (inComposition && timeoutMs--)
            usleep(1000);

        if (timeoutMs == 0) {
            ALOGE("wait for last composePlayThread done timeout");
            return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
        }
    }

    /* Read the pipe again to remove any stale data before triggering a new play */
    nfd = epoll_wait(epollfd, &events, 1, 0);
    if (nfd == -1 && (errno != EINTR)) {
        ALOGE("Failed to wait sleep playLengthMs, error=%d", errno);
        return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    }
    if (nfd > 0) {
        ALOGD("A stale event is cached in the pipe, remove it");
        read(pipefd[0], &status, sizeof(int));
    }

    inComposition = true;
    composeThread = std::thread(composePlayThread, this, composite, callback);
    composeThread.detach();

    ALOGD("trigger composition successfully");
    return ndk::ScopedAStatus::ok();
}

bool Vibrator::isAw() {
    return ledVib.mDetected && (ledVib.vibrator_dev & VIB_AW);
}

/*
 * Awinic playback: every request (on, perform, compose) becomes a job of
 * steps that awPlayThread plays and times. A new job or off() bumps
 * mAwGeneration, which ends the current job at its next wait; that job then
 * stops the motor and reports completion. All aw sysfs writes happen under
 * mAwLock, so a trigger can never land after the off() that should stop it.
 */
void Vibrator::awSubmit(AwJob& job) {
    std::shared_ptr<IVibratorCallback> superseded;

    {
        std::lock_guard<std::mutex> lock(mAwLock);
        /* a queued job that never started still gets its completion */
        if (mAwJobQueued)
            superseded = mAwJob.callback;
        mAwJob = std::move(job);
        mAwJobQueued = true;
        mAwGeneration++;
    }
    mAwCv.notify_all();

    if (superseded != nullptr && !superseded->onComplete().isOk())
        ALOGE("Failed to call onComplete");
}

void Vibrator::awPlayThread(Vibrator *vibrator) {
    std::unique_lock<std::mutex> lock(vibrator->mAwLock);

    while (!vibrator->mAwExit) {
        if (!vibrator->mAwJobQueued) {
            vibrator->mAwCv.wait(lock);
            continue;
        }

        AwJob job = std::move(vibrator->mAwJob);
        vibrator->mAwJob = AwJob();
        vibrator->mAwJobQueued = false;
        uint64_t generation = vibrator->mAwGeneration;
        auto cancelled = [vibrator, generation] {
            return vibrator->mAwExit || vibrator->mAwGeneration != generation;
        };
        bool interrupted = false;
        int ret;

        for (const AwStep& step : job.steps) {
            if (step.delayMs > 0 &&
                vibrator->mAwCv.wait_for(lock, std::chrono::milliseconds(step.delayMs),
                                         cancelled)) {
                interrupted = true;
                break;
            }
            if (cancelled()) {
                interrupted = true;
                break;
            }

            if (step.wave != 0) {
                uint8_t gain = job.useAmplitude ? vibrator->mAwAmplitudeGain : step.gain;

                vibrator->mAwStopped = false;
                if (step.loop)
                    ret = vibrator->ledVib.awPlayLoop(step.lengthMs, gain);
                else
                    ret = vibrator->ledVib.awPlayRam(step.wave, gain);
                if (ret < 0) {
                    interrupted = true;
                    break;
                }
                vibrator->mAwOnActive = job.useAmplitude;
            }

            if (step.lengthMs > 0 &&
                vibrator->mAwCv.wait_for(lock, std::chrono::milliseconds(step.lengthMs),
                                         cancelled)) {
                interrupted = true;
                break;
            }
        }
        vibrator->mAwOnActive = false;

        /*
         * A job cut short (a newer request, a failed write) may leave a
         * waveform playing, and a loop is stopped here besides the driver
         * timer. A RAM sequence that played to its end has already stopped;
         * after off() the motor is stopped already.
         */
        if ((interrupted || job.stopAtEnd) && !vibrator->mAwStopped) {
            vibrator->ledVib.off();
            vibrator->mAwStopped = true;
        }

        if (job.callback != nullptr) {
            lock.unlock();
            ALOGD("Notifying aw playback complete");
            if (!job.callback->onComplete().isOk())
                ALOGE("Failed to call onComplete");
            lock.lock();
        }
    }
}

ndk::ScopedAStatus Vibrator::awOff() {
    std::shared_ptr<IVibratorCallback> dropped;
    int ret;

    {
        std::lock_guard<std::mutex> lock(mAwLock);
        if (mAwJobQueued) {
            dropped = mAwJob.callback;
            mAwJob = AwJob();
            mAwJobQueued = false;
        }
        mAwGeneration++;
        mAwOnActive = false;
        /* always written, whatever the HAL believes the motor is doing */
        ret = ledVib.off();
        mAwStopped = true;
    }
    mAwCv.notify_all();

    if (dropped != nullptr && !dropped->onComplete().isOk())
        ALOGE("Failed to call onComplete");

    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::awOn(int32_t timeoutMs,
                                  const std::shared_ptr<IVibratorCallback>& callback) {
    static const uint8_t shortWaves[] = { 3, 2 };    /* shortest first; wave 1 feels cheap */
    AwStep step = {};
    AwJob job;

    if (timeoutMs <= 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));

    if (timeoutMs <= AW_SHORT_ON_MAX_MS) {
        /*
         * A short pulse plays the shortest RAM waveform that covers it, or
         * wave 2 above its length (boost mode, crisper than a few looped
         * cycles). Completion is reported once both the request and the
         * waveform are over.
         */
        step.wave = 2;
        for (uint8_t wave : shortWaves) {
            if (kAwWaveMs[wave] >= timeoutMs) {
                step.wave = wave;
                break;
            }
        }
        step.lengthMs = std::max(timeoutMs, awWavePlayMs(step.wave));
    } else {
        step.wave = AW_LOOP_WAVE;
        step.loop = true;
        step.lengthMs = timeoutMs;
    }

    job.steps.push_back(step);
    job.useAmplitude = true;
    job.stopAtEnd = step.loop;
    job.callback = callback;
    ALOGD("AwVibrator on %d ms: wave %d%s", timeoutMs, step.wave, step.loop ? " looped" : "");
    awSubmit(job);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::awPerform(Effect effect, EffectStrength es,
                                       const std::shared_ptr<IVibratorCallback>& callback,
                                       int32_t* _aidl_return) {
    const AwEffect *e = awFindEffect(effect);
    AwStep step = {};
    AwJob job;
    int32_t lengthMs = 0;
    int strength;

    if (e == nullptr)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    switch (es) {
    case EffectStrength::LIGHT:
        strength = 0;
        break;
    case EffectStrength::MEDIUM:
        strength = 1;
        break;
    case EffectStrength::STRONG:
        strength = 2;
        break;
    default:
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    step.wave = e->wave;
    step.gain = e->gain[strength];
    step.lengthMs = awWavePlayMs(e->wave);
    job.steps.push_back(step);
    if (e->repeatGapMs > 0) {
        step.delayMs = e->repeatGapMs;
        job.steps.push_back(step);
    }
    for (const AwStep& s : job.steps)
        lengthMs += s.delayMs + s.lengthMs;

    job.useAmplitude = false;
    job.stopAtEnd = false;
    job.callback = callback;
    ALOGD("AwVibrator perform effect %d strength %d: wave %d gain 0x%02x, %d ms",
          static_cast<int>(effect), static_cast<int>(es), step.wave, step.gain, lengthMs);
    awSubmit(job);

    *_aidl_return = lengthMs;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::awSetAmplitude(float amplitude) {
    uint8_t gain;
    int ret = 0;

    /* also rejects NaN */
    if (!(amplitude > 0.0f && amplitude <= 1.0f))
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));

    gain = static_cast<uint8_t>(std::lround(amplitude * AW_GAIN_MAX));
    if (gain < 1)
        gain = 1;

    {
        std::lock_guard<std::mutex> lock(mAwLock);
        mAwAmplitudeGain = gain;
        /* applies from the next on(), or right away while one plays */
        if (mAwOnActive)
            ret = ledVib.awSetGain(gain);
    }

    if (ret < 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
    return ndk::ScopedAStatus::ok();
}

/*
 * Compositions are timed here, one RAM waveform per primitive: the chip's
 * sequencer slots cannot hold silences through the driver ("seq" only accepts
 * waveform numbers), and the gain is one register, so per-primitive scale and
 * delays need a trigger each.
 */
ndk::ScopedAStatus Vibrator::awCompose(const std::vector<CompositeEffect>& composite,
                                       const std::shared_ptr<IVibratorCallback>& callback) {
    AwJob job;

    if (composite.empty() || composite.size() > static_cast<size_t>(ComposeSizeMax))
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);

    for (const CompositeEffect& e : composite) {
        const AwPrimitive *p = awFindPrimitive(e.primitive);
        AwStep step = {};

        if (e.delayMs < 0 || e.delayMs > ComposeDelayMaxMs)
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        /* also rejects NaN */
        if (!(e.scale >= 0.0f && e.scale <= 1.0f))
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        if (p == nullptr)
            return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);

        step.delayMs = e.delayMs;
        step.gain = static_cast<uint8_t>(std::lround(e.scale * p->fullGain));
        /* NOOP or scale 0: keep the timing, play nothing */
        step.wave = step.gain != 0 ? p->wave : 0;
        step.lengthMs = awWavePlayMs(p->wave);
        job.steps.push_back(step);
    }

    job.useAmplitude = false;
    job.stopAtEnd = false;
    job.callback = callback;
    ALOGD("AwVibrator compose %zu primitives", composite.size());
    awSubmit(job);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedAlwaysOnEffects(std::vector<Effect>* _aidl_return __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::alwaysOnEnable(int32_t id __unused, Effect effect __unused,
                                            EffectStrength strength __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::alwaysOnDisable(int32_t id __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getResonantFrequency(float *resonantFreqHz __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getQFactor(float *qFactor __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getFrequencyResolution(float *freqResolutionHz __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getFrequencyMinimum(float *freqMinimumHz __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getBandwidthAmplitudeMap(std::vector<float> *_aidl_return __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getPwlePrimitiveDurationMax(int32_t *durationMs __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getPwleCompositionSizeMax(int32_t *maxSize __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::getSupportedBraking(std::vector<Braking> *supported __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus Vibrator::composePwle(const std::vector<PrimitivePwle> &composite __unused,
                           const std::shared_ptr<IVibratorCallback> &callback __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl

