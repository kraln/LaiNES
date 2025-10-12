#pragma once
#include "common.hpp"

namespace Cartridge {


template <bool wr> u8     access(u16 addr, u8 v = 0);
template <bool wr> u8 chr_access(u16 addr, u8 v = 0);
void signal_scanline(int scanline);
void load(const char* fileName);
void reset();
bool loaded();

// Expansion audio support
void run_mapper_audio(int elapsed);
void end_mapper_audio_frame(int elapsed);

// Mapper IRQ support
bool check_mapper_irq(int elapsed);  // Check if mapper IRQ should be active at given time


}
