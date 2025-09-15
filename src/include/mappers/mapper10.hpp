#pragma once
#include "mapper.hpp"


class Mapper10 : public Mapper
{
    u8 prg_bank;
    u8 chr_bank_fd[2];
    u8 chr_bank_fe[2];
    u8 chr_latch[2];
    u8 mirroring;

    void apply();

    public:
    Mapper10(u8* rom) : Mapper(rom)
    {
        prg_bank = 0;
        chr_bank_fd[0] = chr_bank_fd[1] = 0;
        chr_bank_fe[0] = chr_bank_fe[1] = 0;
        chr_latch[0] = chr_latch[1] = 1;  /* Latch states are 0 or 1, start at FE state */
        mirroring = 0;
        apply();
    }

    u8 write(u16 addr, u8 v);
    u8 chr_read(u16 addr);
    u8 chr_write(u16 addr, u8 v);
};