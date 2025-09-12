#include "shm_debug.hpp"
#include "cpu.hpp"
#include "ppu.hpp"
#include <iostream>

namespace ShmDebug {

SharedMemory* shm_ptr = nullptr;
int shm_fd = -1;
bool shm_enabled = false;

void init(const std::string& shm_name) {
    // First, try to unlink any existing shared memory segment
    shm_unlink(shm_name.c_str());
    
    // Create shared memory object
    shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to create shared memory object\n";
        return;
    }
    
    // Set size of shared memory
    if (ftruncate(shm_fd, sizeof(SharedMemory)) == -1) {
        std::cerr << "Failed to set shared memory size\n";
        close(shm_fd);
        shm_unlink(shm_name.c_str());
        return;
    }
    
    // Map shared memory
    shm_ptr = (SharedMemory*)mmap(0, sizeof(SharedMemory), 
                                  PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        std::cerr << "Failed to map shared memory\n";
        close(shm_fd);
        shm_unlink(shm_name.c_str());
        shm_ptr = nullptr;
        return;
    }
    
    // Initialize shared memory
    memset(shm_ptr, 0, sizeof(SharedMemory));
    // Set controller inputs to "no override" value
    shm_ptr->controller1_input = 0xFF;
    shm_ptr->controller2_input = 0xFF;
    shm_enabled = true;
    
    std::cout << "Shared memory debug interface enabled: " << shm_name << "\n";
}

void cleanup(const std::string& shm_name) {
    if (shm_ptr) {
        munmap(shm_ptr, sizeof(SharedMemory));
        shm_ptr = nullptr;
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_fd = -1;
    }
    // Unlink the shared memory segment to clean it up
    shm_unlink(shm_name.c_str());
    shm_enabled = false;
}

void update_cpu_state() {
    if (!shm_enabled || !shm_ptr) return;
    
    // Copy CPU RAM (first 2KB)
    memcpy(shm_ptr->cpu_ram, CPU::ram, sizeof(CPU::ram));
    
    // Copy CPU registers
    shm_ptr->cpu_a = CPU::A;
    shm_ptr->cpu_x = CPU::X;
    shm_ptr->cpu_y = CPU::Y;
    shm_ptr->cpu_s = CPU::S;
    shm_ptr->cpu_pc = CPU::PC;
    shm_ptr->cpu_flags = CPU::P.get();
    
    // Snapshot the entire CPU memory space efficiently
    CPU::snapshot_memory(shm_ptr->cpu_memory);
}

void update_ppu_state() {
    if (!shm_enabled || !shm_ptr) return;
    
    // Copy PPU nametable RAM
    memcpy(shm_ptr->ppu_ram, PPU::ciRam, sizeof(PPU::ciRam));
    
    // Copy palette RAM
    memcpy(shm_ptr->ppu_palette, PPU::cgRam, sizeof(PPU::cgRam));
    
    // Copy OAM
    memcpy(shm_ptr->ppu_oam, PPU::oamMem, sizeof(PPU::oamMem));
}

}