#include "gui.hpp"
#include "config.hpp"
#include "cartridge.hpp"
#include "shm_debug.hpp"
#include <cstring>
#include <iostream>

int main(int argc, char *argv[])
{
    // Parse command line arguments
    bool enable_shm = false;
    std::string shm_name = "/laines_debug";
    std::string rom_path;
    
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--shm") == 0) {
            enable_shm = true;
        } else if (strcmp(argv[i], "--shm-name") == 0 && i + 1 < argc) {
            shm_name = argv[++i];
            enable_shm = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            std::cout << "Usage: " << argv[0] << " [options] [rom_file]\n"
                      << "Options:\n"
                      << "  --shm           Enable shared memory debug interface\n"
                      << "  --shm-name NAME Set shared memory name (default: /laines_debug)\n"
                      << "  --help, -h      Show this help message\n"
                      << "Arguments:\n"
                      << "  rom_file        Path to NES ROM file to load at startup\n";
            return 0;
        } else if (argv[i][0] != '-') {
            // Assume it's a ROM file path
            rom_path = argv[i];
        }
    }
    
    // Initialize shared memory if requested
    if (enable_shm) {
        ShmDebug::init(shm_name);
    }
    
    GUI::load_settings();
    GUI::init();
    
    // Load ROM if specified
    if (!rom_path.empty()) {
        Cartridge::load(rom_path.c_str());
        // If ROM loaded successfully, start unpaused
        if (Cartridge::loaded()) {
            GUI::set_paused(false);
        }
    }
    
    GUI::run();
    
    // Cleanup shared memory
    if (enable_shm) {
        ShmDebug::cleanup(shm_name);
    }

    return 0;
}
