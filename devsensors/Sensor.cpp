/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Sensor.h"

#include <hardware/sensors.h>
#include <utils/SystemClock.h>

#include <cmath>


#include <fcntl.h>
#include <linux/input.h>
#include <log/log.h>
#include <fstream> 

#include <iostream>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

using ::android::hardware::sensors::V1_0::MetaDataEventType;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V1_0::SensorStatus;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::SensorType;

Sensor::Sensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : mIsEnabled(false),
      mSamplingPeriodNs(0),
      mLastSampleTimeNs(0),
      mCallback(callback),
      mMode(OperationMode::NORMAL) {
    mSensorInfo.sensorHandle = sensorHandle;
    mSensorInfo.vendor = "Vendor String";
    mSensorInfo.version = 1;
    constexpr float kDefaultMaxDelayUs = 1000 * 1000;
    mSensorInfo.maxDelay = kDefaultMaxDelayUs;
    mSensorInfo.fifoReservedEventCount = 0;
    mSensorInfo.fifoMaxEventCount = 0;
    mSensorInfo.requiredPermission = "";
    mSensorInfo.flags = 0;
    mRunThread = std::thread(startThread, this);
}

Sensor::~Sensor() {
    // Ensure that lock is unlocked before calling mRunThread.join() or a
    // deadlock will occur.
    {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mStopThread = true;
        mIsEnabled = false;
        mWaitCV.notify_all();
    }
    mRunThread.join();
}

const SensorInfo& Sensor::getSensorInfo() const {
    return mSensorInfo;
}

void Sensor::batch(int64_t samplingPeriodNs) {
    samplingPeriodNs = std::clamp(samplingPeriodNs,
                                  static_cast<int64_t>(mSensorInfo.minDelay) * 1000,
                                  static_cast<int64_t>(mSensorInfo.maxDelay) * 1000);

    if (mSamplingPeriodNs != samplingPeriodNs) {
        mSamplingPeriodNs = samplingPeriodNs;

        std::string msg = "LightSensor: samplingPeriodNs: " + std::to_string(samplingPeriodNs);
        ALOGD("%s", msg.c_str());
        // Wake up the 'run' thread to check if a new event should be generated now
        mWaitCV.notify_all();
    }
}

void Sensor::activate(bool enable) {
    if (mIsEnabled != enable) {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mIsEnabled = enable;
        mWaitCV.notify_all();
    }
}

Result Sensor::flush() {
    // Only generate a flush complete event if the sensor is enabled and if the sensor is not a
    // one-shot sensor.
    if (!mIsEnabled || (mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::ONE_SHOT_MODE))) {
        return Result::BAD_VALUE;
    }

    // Note: If a sensor supports batching, write all of the currently batched events for the sensor
    // to the Event FMQ prior to writing the flush complete event.
    Event ev;
    ev.sensorHandle = mSensorInfo.sensorHandle;
    ev.sensorType = SensorType::META_DATA;
    ev.u.meta.what = MetaDataEventType::META_DATA_FLUSH_COMPLETE;
    std::vector<Event> evs{ev};
    mCallback->postEvents(evs, isWakeUpSensor());

    return Result::OK;
}

void Sensor::startThread(Sensor* sensor) {
    sensor->run();
}

void Sensor::run() {
    std::unique_lock<std::mutex> runLock(mRunMutex);
    constexpr int64_t kNanosecondsInSeconds = 1000 * 1000 * 1000;

    while (!mStopThread) {
        if (!mIsEnabled || mMode == OperationMode::DATA_INJECTION) {
            mWaitCV.wait(runLock, [&] {
                return ((mIsEnabled && mMode == OperationMode::NORMAL) || mStopThread);
            });
        } else {
            timespec curTime;
            clock_gettime(CLOCK_REALTIME, &curTime);
            int64_t now = (curTime.tv_sec * kNanosecondsInSeconds) + curTime.tv_nsec;
            int64_t nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;

            if (now >= nextSampleTime) {
                mLastSampleTimeNs = now;
                nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;
                mCallback->postEvents(readEvents(), isWakeUpSensor());
            }

            mWaitCV.wait_for(runLock, std::chrono::nanoseconds(nextSampleTime - now));
        }
    }
}

bool Sensor::isWakeUpSensor() {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::WAKE_UP);
}

std::vector<Event> Sensor::readEvents() {
    std::vector<Event> events;
    Event event;
    event.sensorHandle = mSensorInfo.sensorHandle;
    event.sensorType = mSensorInfo.type;
    event.timestamp = ::android::elapsedRealtimeNano();
    event.u.vec3.x = 0;
    event.u.vec3.y = 0;
    event.u.vec3.z = 0;
    event.u.vec3.status = SensorStatus::ACCURACY_HIGH;
    events.push_back(event);
    return events;
}

void Sensor::setOperationMode(OperationMode mode) {
    if (mMode != mode) {
        std::unique_lock<std::mutex> lock(mRunMutex);
        mMode = mode;
        mWaitCV.notify_all();
    }
}

bool Sensor::supportsDataInjection() const {
    return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::DATA_INJECTION);
}

Result Sensor::injectEvent(const Event& event) {
    Result result = Result::OK;
    if (event.sensorType == SensorType::ADDITIONAL_INFO) {
        // When in OperationMode::NORMAL, SensorType::ADDITIONAL_INFO is used to push operation
        // environment data into the device.
    } else if (!supportsDataInjection()) {
        result = Result::INVALID_OPERATION;
    } else if (mMode == OperationMode::DATA_INJECTION) {
        mCallback->postEvents(std::vector<Event>{event}, isWakeUpSensor());
    } else {
        result = Result::BAD_VALUE;
    }
    return result;
}

OnChangeSensor::OnChangeSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : Sensor(sensorHandle, callback), mPreviousEventSet(false) {
    mSensorInfo.flags |= SensorFlagBits::ON_CHANGE_MODE;
}

void OnChangeSensor::activate(bool enable) {
    Sensor::activate(enable);
    if (!enable) {
        mPreviousEventSet = false;
    }
}

std::vector<Event> OnChangeSensor::readEvents() {
    //ALOGE("READ EVENT");
    std::vector<Event> events = Sensor::readEvents();
    std::vector<Event> outputEvents;

    for (auto iter = events.begin(); iter != events.end(); ++iter) {
        Event ev = *iter;
        if (ev.u.vec3 != mPreviousEvent.u.vec3 || !mPreviousEventSet) {
            outputEvents.push_back(ev);
            mPreviousEvent = ev;
            mPreviousEventSet = true;
        }
    }
    return outputEvents;
}

ContinuousSensor::ContinuousSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : Sensor(sensorHandle, callback) {
    mSensorInfo.flags |= SensorFlagBits::CONTINUOUS_MODE;
}

LightSensor::LightSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
    : OnChangeSensor(sensorHandle, callback),
         mInputReader((size_t)(4)) 
    {
    mSensorInfo.name = "Smartlamp Light Sensor ";
    mSensorInfo.vendor = "devtitans";
    mSensorInfo.type = SensorType::LIGHT;
    mSensorInfo.typeAsString = SENSOR_STRING_TYPE_LIGHT;
    mSensorInfo.maxRange = 43000.0f;
    mSensorInfo.resolution = 10.0f;
    mSensorInfo.power = 0.001f;         // mA
    mSensorInfo.minDelay = 200 * 1000;  // microseconds
    
}

int LightSensor::enable(bool enable){
    std::ofstream in("/sys/class/input/"+std::string(input_name)+"/device/enable", std::ios::trunc); // input

    int inputValue = enable ? 1 : 0;
    
    if(in) {
        in << inputValue;
        in.close();

        std::string  enable_states[2] = {"disabled", "enabled"};
        std::string msg = "Lightsensor is " + enable_states[enable] + " event: " + input_name;
        ALOGD("%s", msg.c_str());

        return 1;
    } else{
        ALOGD("LightSensor cannot be enabled (unable to access enable file) %s", input_name);
        return 0;
    }
}

/*void LightSensor::batch(int64_t samplingPeriodNs) {
    Sensor::batch(samplingPeriodNs);
    //send mSamplingPeriodNs to sensor
}*/

void LightSensor::activate(bool enable) {
    OnChangeSensor::activate(enable);
    //std::ofstream enable_s("/sys/kernel/smartlamp/enable");
    //std::ofstream polldelay_s("/sys/kernel/smartlamp/polldelay");

    if(enable){
        ifd = LightSensor::openInput("smartlamp");
        ALOGD("LightSensor is active - poll time %ld !!!!!!!!!!!!!", (signed long int) mSamplingPeriodNs);
        ALOGD("LightSensor is active - ifd %d !!!!!!!!!!!!!", ifd);
        LightSensor::enable(enable);
        //enable_s << 1;
        //enable_s.close();
    } else {
        ALOGE("LightSensor is inactive !!!!!!!!!!!!!");
        LightSensor::enable(enable);
        //enable_s << 0;
        //enable_s.close();
        close(ifd);
    }
}

int LightSensor::openInput(const char* inputName) {
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;
    dir = opendir(dirname);
    if(dir == NULL)
        return -1;
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while((de = readdir(dir))) {
        ALOGD("LightSensor Looking for input '%s' input device raw", de->d_name);
        if(de->d_name[0] == '.' &&
                (de->d_name[1] == '\0' ||
                        (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        if (fd>=0) {
            char name[80];
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                name[0] = '\0';
            }
            ALOGD("LightSensor: Looking for input '%s' input device", name);
            if (!strcmp(name, inputName)) {
                strcpy(input_name, filename);
                ALOGD("LightSensor: choosed '%s' as input device", name);
                break;
            } else {
                close(fd);
                fd = -1;
            }
        }
    }
    closedir(dir);
    ALOGE_IF(fd<0, "LightSensor couldn't find '%s' input device", inputName);
    return fd;
}

std::vector<Event> LightSensor::readEvents() {
    
    std::vector<Event> events;

    input_event const* event;
    sensors_event_t mPendingEvent;
    ssize_t s = 0;

    
    mPendingEvent.timestamp = ::android::elapsedRealtimeNano();

    if(ifd != 0){
        //std::ifstream in("/sys/kernel/smartlamp/ldr"); // input
          
        s = mInputReader.fill(ifd);

        /*if(in) {
            in >> sensorRead; 
        } */
    }


    int count = 4;

    while(s !=0 && count && mInputReader.readEvent(&event)){
        int type = event->type;
		ALOGD("LightSensor readEvents() - event->type is: %d", type);
		if (type == EV_ABS) {
			if (event->code == ABS_LIGHT) {
                mPendingEvent.type = type;
                mPendingEvent.light = (float) event->value;
                sensorRead = mPendingEvent.light;
            }
        }else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            count--;
        }

        mInputReader.next();
    }

    ALOGE("LightSensor: READ EVENT %f", sensorRead);

    Event event_s;
    event_s.sensorHandle = mSensorInfo.sensorHandle;
    event_s.sensorType = mSensorInfo.type;
    event_s.timestamp = mPendingEvent.timestamp;
    event_s.u.scalar = sensorRead;
    events.push_back(event_s);

    std::vector<Event> outputEvents;

    // Implementing on-change sensor behavior (maybe not needed due to input device event code)
    for (auto iter = events.begin(); iter != events.end(); ++iter) {
        Event ev = *iter;
        if (ev.u.vec3 != mPreviousEvent.u.vec3 || !mPreviousEventSet) {
            outputEvents.push_back(ev);
            mPreviousEvent = ev;
            mPreviousEventSet = true;
        }
    }
    return outputEvents;
}

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
