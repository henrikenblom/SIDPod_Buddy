#include <Arduino.h>
#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"

AudioInfo info44k1(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
I2SStream i2s;

constexpr int BYTES_PER_FRAME = 4;

int32_t get_sound_data(Frame *data, const int32_t frameCount) {
    return static_cast<int32_t>(i2s.readBytes(reinterpret_cast<uint8_t *>(data), frameCount * BYTES_PER_FRAME)) /
           BYTES_PER_FRAME;
}

void avoidWatchdogReboots() {
    delay(1000);
}

void setup() {
    setCpuFrequencyMhz(80);
    Serial.begin(115200);
    auto cfg = i2s.defaultConfig(RX_MODE);
    cfg.i2s_format = I2S_STD_FORMAT;
    cfg.is_master = false;
    cfg.set(info44k1);
    i2s.begin(cfg);

    a2dp_source.start("84-1511", get_sound_data);
}

void loop() {
    avoidWatchdogReboots();
}
