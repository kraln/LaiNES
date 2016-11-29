#pragma once
#include "mapper.hpp"

class Mapper34 : public Mapper
{
    bool is_nina;  // true for NINA-001, false for BNROM
    u8 prg_bank;
    u8 chr_bank[2];

    void apply();

  public:
    Mapper34(u8* rom) : Mapper(rom)
    {
        // Detect variant: NINA-001 has CHR-ROM, BNROM has CHR-RAM
        is_nina = (chrSize > 0);

        prg_bank = 0;
        chr_bank[0] = 0;
        chr_bank[1] = 0;
        apply();
    }

    u8 write(u16 addr, u8 v);
    u8 chr_write(u16 addr, u8 v);
};