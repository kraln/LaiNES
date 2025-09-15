#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <signal.h>
#include <chrono>
#include <thread>

// Shared memory structure matching shm_debug.hpp
struct SharedMemory {
    uint8_t cpu_ram[0x800];
    uint8_t cpu_a;
    uint8_t cpu_x;
    uint8_t cpu_y;
    uint8_t cpu_s;
    uint16_t cpu_pc;
    uint8_t cpu_flags;
    uint8_t ppu_ram[0x800];
    uint8_t ppu_palette[0x20];
    uint8_t ppu_oam[0x100];
    uint8_t cpu_memory[0x10000];  // Full CPU memory space
    volatile uint8_t controller1_input;
    volatile uint8_t controller2_input;
    volatile bool update_request;
    volatile bool update_complete;
    volatile bool rom_loaded;
};

// AccuracyCoin test result addresses (from AccuracyCoin.asm)
struct TestResult {
    const char* name;
    uint16_t addr;
};

TestResult accuracyCoinTests[] = {
    // CPU Behavior tests - 10 tests
    {"ROM is not writable", 0x405},
    {"RAM Mirroring", 0x403},
    {"PC Wraparound", 0x44D},
    {"The Decimal Flag", 0x474},
    {"The B Flag", 0x475},
    {"Dummy read cycles", 0x406},
    {"Dummy write cycles", 0x407},
    {"Open Bus", 0x408},
    {"Unofficial Instructions", 0x402},
    {"All NOP instructions", 0x47D},
    
    // Addressing mode wraparound - 6 tests
    {"Absolute Indexed", 0x46E},
    {"Zero Page Indexed", 0x46F},
    {"Indirect", 0x470},
    {"Indirect, X", 0x471},
    {"Indirect, Y", 0x472},
    {"Relative", 0x473},
    
    // Unofficial Instructions: SLO - 7 tests
    {"SLO $03", 0x409},
    {"SLO $07", 0x40A},
    {"SLO $0F", 0x40B},
    {"SLO $13", 0x40C},
    {"SLO $17", 0x40D},
    {"SLO $1B", 0x40E},
    {"SLO $1F", 0x40F},
    
    // Unofficial Instructions: RLA - 7 tests
    {"RLA $23", 0x419},
    {"RLA $27", 0x41A},
    {"RLA $2F", 0x41B},
    {"RLA $33", 0x41C},
    {"RLA $37", 0x41D},
    {"RLA $3B", 0x41E},
    {"RLA $3F", 0x41F},
    
    // Unofficial Instructions: SRE - 7 tests
    {"SRE $43", 0x420},
    {"SRE $47", 0x47F},  // Special address!
    {"SRE $4F", 0x422},
    {"SRE $53", 0x423},
    {"SRE $57", 0x424},
    {"SRE $5B", 0x425},
    {"SRE $5F", 0x426},
    
    // Unofficial Instructions: RRA - 7 tests  
    {"RRA $63", 0x427},
    {"RRA $67", 0x428},
    {"RRA $6F", 0x429},
    {"RRA $73", 0x42A},
    {"RRA $77", 0x42B},
    {"RRA $7B", 0x42C},
    {"RRA $7F", 0x42D},
    
    // Unofficial Instructions: *AX - 10 tests
    {"SAX $83", 0x42E},
    {"SAX $87", 0x42F},
    {"SAX $8F", 0x430},
    {"SAX $97", 0x431},
    {"LAX $A3", 0x432},
    {"LAX $A7", 0x433},
    {"LAX $AF", 0x434},
    {"LAX $B3", 0x435},
    {"LAX $B7", 0x436},
    {"LAX $BF", 0x437},
    
    // Unofficial Instructions: DCP - 7 tests
    {"DCP $C3", 0x438},
    {"DCP $C7", 0x439},
    {"DCP $CF", 0x43A},
    {"DCP $D3", 0x43B},
    {"DCP $D7", 0x43C},
    {"DCP $DB", 0x43D},
    {"DCP $DF", 0x43E},
    
    // Unofficial Instructions: ISC - 7 tests
    {"ISC $E3", 0x43F},
    {"ISC $E7", 0x440},
    {"ISC $EF", 0x441},
    {"ISC $F3", 0x442},
    {"ISC $F7", 0x443},
    {"ISC $FB", 0x444},
    {"ISC $FF", 0x445},
    
    // Unofficial Instructions: SH* - 6 tests
    {"SHA $93", 0x446},
    {"SHA $9F", 0x447},
    {"SHS $9B", 0x448},
    {"SHY $9C", 0x449},
    {"SHX $9E", 0x44A},
    {"LAE $BB", 0x44B},
    
    // Unofficial Instructions: Immediates - 8 tests
    {"ANC $0B", 0x410},
    {"ANC $2B", 0x411},
    {"ASR $4B", 0x412},
    {"ARR $6B", 0x413},
    {"ANE $8B", 0x414},
    {"LXA $AB", 0x415},
    {"AXS $CB", 0x416},
    {"SBC $EB", 0x417},
    
    // CPU Interrupts - 3 tests
    {"Interrupt flag latency", 0x461},
    {"NMI Overlap BRK", 0x462},
    {"NMI Overlap IRQ", 0x463},
    
    // APU Registers and DMA tests - 10 tests
    {"DMA + Open Bus", 0x46C},
    {"DMA + $2007 Read", 0x44C},
    {"DMA + $2007 Write", 0x44F},
    {"DMA + $4015 Read", 0x45D},
    {"DMA + $4016 Read", 0x45E},
    {"APU Register Activation", 0x45C},
    {"DMC DMA Bus Conflicts", 0x46B},
    {"DMC DMA + OAM DMA", 0x477},
    {"Explicit DMA Abort", 0x479},
    {"Implicit DMA Abort", 0x478},
    
    // APU Tests - 8 tests
    {"Length Counter", 0x465},
    {"Length Table", 0x466},
    {"Frame Counter IRQ", 0x467},
    {"Frame Counter 4-step", 0x468},
    {"Frame Counter 5-step", 0x469},
    {"Delta Modulation Channel", 0x46A},
    {"Controller Strobing", 0x45F},
    {"Controller Clocking", 0x47A},
    
    // Power On State - 5 tests (skipping first one)
    {"PPU Reset Flag", 0x418},  // PowerOn_PPUReset
    // CPU RAM at 0x3FC is on page 3, omitted from all-test-result-table
    // CPU Registers at 0x370-374 are also on page 3, omitted
    // PPU RAM at 0x320 is on page 3, omitted  
    // Palette RAM at 0x340 is on page 3, omitted
    
    // PPU Registers - 4 tests
    {"PPU Register Mirroring", 0x404},
    {"PPU Register Open Bus", 0x44E},
    {"PPU Read Buffer", 0x476},
    {"Palette RAM Quirks", 0x47E},
    
    // PPU NMI and VBlank - 7 tests
    {"VBlank beginning", 0x450},
    {"VBlank end", 0x451},
    {"NMI Control", 0x452},
    {"NMI Timing", 0x453},
    {"NMI Suppression", 0x454},
    {"NMI at VBlank end", 0x455},
    {"NMI disabled at VBlank", 0x456},
    
    // PPU Sprites and OAM - 6 tests
    {"Sprite overflow behavior", 0x459},
    {"Sprite 0 Hit behavior", 0x457},
    {"Arbitrary Sprite zero", 0x458},
    {"Misaligned OAM behavior", 0x45A},
    {"Address $2004 behavior", 0x45B},
    {"OAM Corruption", 0x47B},
    
    // Miscellaneous - 3 tests
    {"RMW $2007 Extra Write", 0x464},
    {"Instruction Timing", 0x460},
    {"Implied Dummy Reads", 0x46D},
    {"JSR Edge Cases", 0x47C}
};

const int NUM_TESTS = sizeof(accuracyCoinTests) / sizeof(TestResult);

const char* getErrorDescription(uint8_t code) {
    if (code == 0 || code == 1 || code == 2) return "PASS";
    if (code == 0xFF) return "NOT RUN";
    
    // Return hex code for errors
    static char buf[16];
    snprintf(buf, sizeof(buf), "FAIL %02X", code);
    return buf;
}

// Controller button bits
enum Controller {
    BTN_A      = 0x01,
    BTN_B      = 0x02,
    BTN_SELECT = 0x04,
    BTN_START  = 0x08,
    BTN_UP     = 0x10,
    BTN_DOWN   = 0x20,
    BTN_LEFT   = 0x40,
    BTN_RIGHT  = 0x80
};

volatile bool keep_running = true;

void signal_handler(int sig) {
    keep_running = false;
}

int main(int argc, char* argv[]) {
    bool auto_mode = false;
    const char* shm_name = "/laines_debug";
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--auto") == 0) {
            auto_mode = true;
        } else if (strcmp(argv[i], "--shm-name") == 0 && i + 1 < argc) {
            shm_name = argv[++i];
        } else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  --auto          Automatically run all tests\n");
            printf("  --shm-name NAME Use specified shared memory name (default: /laines_debug)\n");
            printf("  --help          Show this help message\n");
            return 0;
        }
    }
    
    // Open shared memory
    int shm_fd = shm_open(shm_name, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("Failed to open shared memory");
        printf("Make sure laines is running with --shm flag\n");
        return 1;
    }
    
    // Map shared memory
    SharedMemory* shm = (SharedMemory*)mmap(nullptr, sizeof(SharedMemory),
                                           PROT_READ | PROT_WRITE, MAP_SHARED,
                                           shm_fd, 0);
    if (shm == MAP_FAILED) {
        perror("Failed to map shared memory");
        close(shm_fd);
        return 1;
    }
    
    // Set up signal handler
    signal(SIGINT, signal_handler);
    
    if (auto_mode) {
        printf("=== AccuracyCoin Automatic Test Mode ===\n");
        printf("Waiting for ROM to load...\n");
        
        // Wait for ROM to be loaded
        while (!shm->rom_loaded && keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!keep_running) {
            munmap(shm, sizeof(SharedMemory));
            close(shm_fd);
            return 0;
        }
        
        printf("ROM loaded. Sending controller inputs to run all tests...\n");
        
        // Send controller inputs to navigate menu and run all tests
        // Based on AccuracyCoin's menu structure:
        // Start: Run all tests
        
        // Wait a bit for the ROM to initialize
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Press Start to run all tests
        shm->controller1_input = BTN_START;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        shm->controller1_input = 0xFF;  // Release
        
        printf("Running all tests... (this may take a while)\n");
        
        // Wait for tests to complete by monitoring the RunningAllTests flag
        // This is at address $35 in CPU RAM
        int dots = 0;
        while (keep_running) {
            uint8_t running_all_tests = shm->cpu_ram[0x35];
            
            // Print progress dots
            if (dots++ % 10 == 0) {
                printf(".");
                fflush(stdout);
            }
            
            // Check if tests are complete (RunningAllTests flag cleared)
            if (running_all_tests == 0) {
                printf("\nTests complete!\n");
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    // Display test results
    printf("\n=== AccuracyCoin Test Results ===\n");
    printf("Total tests: %d\n\n", NUM_TESTS);
    
    int passed = 0;
    int failed = 0;
    int not_run = 0;
    
    for (int i = 0; i < NUM_TESTS; i++) {
        uint16_t addr = accuracyCoinTests[i].addr;
        // Read directly from the full CPU memory space
        uint8_t result = shm->cpu_memory[addr];
        
        // Decode the test result
        // AccuracyCoin stores results as: (ErrorCode << 2) | status
        // Where status: 0=PASS behavior 0, 1=PASS behavior 1, 2=FAIL, 3=PASS behavior 2
        uint8_t status_bits = result & 0x03;
        uint8_t error_code = (result & 0xFC) >> 2;
        
        if (result == 0xFF) {
            not_run++;
            printf("\033[33m[----]\033[0m             ");
        } else if (status_bits == 0 || status_bits == 1 || status_bits == 3) {
            // PASS with behavior 0, 1, or 2 (stored as 3)
            passed++;
            printf("\033[32m[PASS]\033[0m ");
            if (status_bits == 1) {
                printf("(behavior 1) ");
            } else if (status_bits == 3) {
                printf("(behavior 2) ");
            } else {
                printf("            ");
            }
        } else if (status_bits == 2) {
            // FAIL with error code
            failed++;
            printf("\033[31m[FAIL %X]\033[0m          ", error_code);
        } else {
            // Shouldn't happen
            failed++;
            printf("\033[31m[????]\033[0m             ");
        }
        
        printf("%-35s (0x%03X)\n", accuracyCoinTests[i].name, addr);
    }
    
    printf("\n=== Summary ===\n");
    printf("Passed:  %d/%d\n", passed, NUM_TESTS);
    printf("Failed:  %d/%d\n", failed, NUM_TESTS);
    printf("Not run: %d/%d\n", not_run, NUM_TESTS);
    printf("Score:   %.1f%%\n", (passed * 100.0) / NUM_TESTS);
    
    // Clean up
    munmap(shm, sizeof(SharedMemory));
    close(shm_fd);
    
    return 0;
}