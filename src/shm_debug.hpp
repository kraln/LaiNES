#pragma once
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include "common.hpp"

namespace ShmDebug {

// Structure for shared memory layout
struct SharedMemory {
    // CPU state
    u8 cpu_ram[0x800];      // CPU RAM (2KB)
    u8 cpu_a, cpu_x, cpu_y, cpu_s;
    u16 cpu_pc;
    u8 cpu_flags;
    
    // PPU state  
    u8 ppu_ram[0x800];      // Nametable RAM (2KB)
    u8 ppu_palette[0x20];   // Palette RAM (32 bytes)
    u8 ppu_oam[0x100];      // OAM (sprite data, 256 bytes)
    
    // Full CPU memory space snapshot (64KB)
    u8 cpu_memory[0x10000];  // Full 64KB address space
    
    // Controller input (can be written by external process)
    volatile u8 controller1_input;  // Bits: A B Select Start Up Down Left Right
    volatile u8 controller2_input;
    
    // Control flags
    volatile bool update_request;
    volatile bool update_complete;
    volatile bool rom_loaded;  // Set to true when ROM is loaded
};

extern SharedMemory* shm_ptr;
extern int shm_fd;
extern bool shm_enabled;

void init(const std::string& shm_name = "/laines_debug");
void cleanup(const std::string& shm_name = "/laines_debug");
void update_cpu_state();
void update_ppu_state();

}