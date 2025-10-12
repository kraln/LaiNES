#pragma once
#include "mapper.hpp"
#include <Nes_Vrc6.h>


class Mapper24 : public Mapper
{
    // VRC6 expansion audio
    Nes_Vrc6 vrc6;

    // PRG banking
    u8 prg_bank_16k;  // 16K bank at $8000-$BFFF
    u8 prg_bank_8k;   // 8K bank at $C000-$DFFF

    // CHR banking
    u8 chr_banks[8];  // Eight 1K banks at $0000-$1FFF

    // IRQ state
    u8 irq_latch;
    u8 irq_counter;
    bool irq_enabled;
    bool irq_mode;           // false = scanline mode, true = cycle mode
    bool irq_enable_after_ack;
    bool irq_active;         // Track whether IRQ line is currently asserted
    int last_cpu_cycle;      // Track last CPU cycle for cycle mode timing

    // VRC6a vs VRC6b (address line differences)
    bool is_vrc6b;

    void apply_prg_banks();
    void apply_chr_banks();
    void clock_irq();           // Clock the IRQ counter (increments/checks counter)

    // Decode register address accounting for VRC6a/b differences
    u16 decode_addr(u16 addr);

  public:
    Mapper24(u8* rom, bool vrc6b = false);

    u8 read(u16 addr) override;
    u8 write(u16 addr, u8 v) override;
    u8 chr_read(u16 addr) override;
    void signal_scanline(int scanline) override;

    // Expansion audio support
    bool has_audio() override { return true; }
    void run_audio(int elapsed) override;
    void end_audio_frame(int elapsed) override;

    // IRQ support
    bool check_irq(int elapsed) override;
};
