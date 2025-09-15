#pragma once
#include "mapper.hpp"


class Mapper11 : public Mapper
{
    u8 regs[1];
    bool vertical_mirroring;
    void apply();

    public:
    Mapper11(u8* rom) : Mapper(rom)
    {
        vertical_mirroring = rom[6] & 0x01;
        regs[0] = 0;
        apply();
    }

    u8 write(u16 addr, u8 v);
    u8 chr_write(u16 addr, u8 v);
};