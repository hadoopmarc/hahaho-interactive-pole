#pragma once

#include <ESP_I2S.h>

void setup_mp3();
void play_mp3(const char filename[]);
void playBuff(i2s_port_t i2s_num, size_t len);
