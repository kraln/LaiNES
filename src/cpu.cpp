#include <cstdlib>
#include <cstring>
#include <iostream>
#include <cstdio>
#include "apu.hpp"
#include "cartridge.hpp"
#include "joypad.hpp"
#include "ppu.hpp"
#include "cpu.hpp"
#include "shm_debug.hpp"

namespace CPU {

/* Note on unofficial opcodes SHA/SHS behavior:
 * Early RP2A03G CPUs (and earlier): High byte corruption ANDs with A & X (Behavior 1)
 * Late RP2A03G CPUs (and later): High byte corruption ANDs with X only (Behavior 2)
 * We implement Behavior 2 as it's more common in later hardware revisions.
 */


/* CPU state */
u8 ram[0x800];
u8 A, X, Y, S;
u16 PC;
Flags P;

// NMI edge detection with 1-instruction delay
bool nmi_line = false;      // Current state of NMI line from PPU
bool nmi_pending = false;   // Edge detected but not yet latched (delays by 1 instruction)
bool nmi_latch = false;     // Ready to trigger interrupt (promoted from pending)
bool nmi_previous = false;  // Previous state for edge detection

bool irq;
u8 data_bus = 0;  // Open bus behavior
bool is_put_cycle = false;  // Track whether current cycle is a put (write) cycle
bool is_rmw_cycle = false;  // Track whether current cycle is part of a RMW instruction

// Remaining clocks to end frame:
const int TOTAL_CYCLES = 29781;
int remainingCycles;
inline int elapsed() { return TOTAL_CYCLES - remainingCycles; }

// Track if we already polled for interrupts this instruction (for special cases like RTI)
bool interrupt_already_polled = false;

/* Cycle emulation */
#define T   tick()
inline void tick() { PPU::step(); PPU::step(); PPU::step(); remainingCycles--; }

/* Flags updating */
inline void upd_cv(u8 x, u8 y, s16 r) { P[C] = (r>0xFF); P[V] = ~(x^y) & (x^r) & 0x80; }
inline void upd_nz(u8 x)              { P[N] = x & 0x80; P[Z] = (x == 0);              }
// Does adding I to A cross a page?
inline bool cross(u16 a, s8 i) { return ((a+i) & 0xFF00) != ((a & 0xFF00)); }
// Overload for unsigned offsets
inline bool cross(u16 a, u8 i) { return ((a+i) & 0xFF00) != ((a & 0xFF00)); }

/* Memory access */
void dma_oam(u8 bank);
template<bool wr> inline u8 access(u16 addr, u8 v = 0)
{
    u8* r;
    u8 result = data_bus;  // Default to open bus
    switch (addr)
    {
        case 0x0000 ... 0x1FFF:  r = &ram[addr % 0x800]; if (wr) *r = v; result = *r; break;  // RAM.
        case 0x2000 ... 0x3FFF:  result = PPU::access<wr>(addr & 7, wr ? v : data_bus, is_rmw_cycle); break;  // PPU.

        // APU:
        case 0x4000 ... 0x4013:
        case            0x4015:          result = APU::access<wr>(elapsed(), addr, wr ? v : data_bus, is_put_cycle); break;
        case            0x4017:  if (wr) result = APU::access<wr>(elapsed(), addr, v, is_put_cycle);
                                 else result = (data_bus & 0xE0) | (Joypad::read_state(1) & 0x1F);  // Joypad 1, bits 5-7 open bus.
                                 break;

        case            0x4014:  if (wr) dma_oam(v); break;                          // OAM DMA.
        case            0x4016:  if (wr) { Joypad::write_strobe(v & 1, elapsed(), is_put_cycle); break; }     // Joypad strobe.
                                 else result = (data_bus & 0xE0) | (Joypad::read_state(0) & 0x1F);  // Joypad 0, bits 5-7 open bus.
                                 break;
        case 0x4018 ... 0x5FFF:  if (wr) result = v; else result = data_bus; break;  // Open bus - return current data_bus value
        case 0x6000 ... 0xFFFF:  result = Cartridge::access<wr>(addr, v); break;              // Cartridge.
    }
    // Update data bus - writes always update it, reads update it except for $4015
    if (wr) {
        data_bus = v;
    } else if (addr != 0x4015) {
        // Update data bus for all reads except $4015 (APU status)
        u8 old_bus = data_bus;
        data_bus = result;
    }
    // Note: Reading from $4015 does NOT update the data bus
    return result;
}
inline u8  wr(u16 a, u8 v)      { T; is_put_cycle = true; u8 result = access<1>(a, v); is_put_cycle = false; return result; }
inline u8  rd(u16 a)            { T; is_put_cycle = false; return access<0>(a);      }
// Dummy read - doesn't update data bus
inline u8  rd_dummy(u16 a)      { T; is_put_cycle = false; u8 old_bus = data_bus; u8 result = access<0>(a); data_bus = old_bus; return result; }

// Efficient bulk memory snapshot for debugging
void snapshot_memory(u8* dest) {
    // Copy RAM directly
    for (int i = 0; i < 0x800; i++) {
        dest[i] = ram[i];
    }
    // Mirror RAM (0x800-0x1FFF)
    for (int i = 0x800; i < 0x2000; i++) {
        dest[i] = ram[i & 0x7FF];
    }
    // PPU registers (0x2000-0x3FFF) - just return last bus value
    for (int i = 0x2000; i < 0x4000; i++) {
        dest[i] = data_bus;
    }
    // APU/IO registers (0x4000-0x401F) - return 0 to avoid side effects
    for (int i = 0x4000; i < 0x4020; i++) {
        dest[i] = 0;
    }
    // Cartridge space (0x4020-0xFFFF) - read without side effects
    for (int i = 0x4020; i < 0x10000; i++) {
        dest[i] = Cartridge::access<0>(i, 0);
    }
}
inline u16 rd16_d(u16 a, u16 b) { return rd(a) | (rd(b) << 8); }  // Read from A and B and merge.
inline u16 rd16(u16 a)          { return rd16_d(a, a+1);       }
inline u8  push(u8 v)           { return wr(0x100 + (S--), v); }
inline u8  pop()                { return rd(0x100 + (++S));    }
void dma_oam(u8 bank) { for (int i = 0; i < 256; i++)  wr(0x2014, rd(bank*0x100 + i)); }

/* Addressing modes */
inline u16 imm()   { return PC++;                                       }
inline u16 imm16() { PC += 2; return PC - 2;                            }
inline u16 abs()   { return rd16(imm16());                              }
inline u16 _abx()  { u16 a = abs(); rd((a & 0xFF00) | ((a + X) & 0xFF)); return a + X; }  // Exception - always dummy read.
inline u16 _aby()  { u16 a = abs(); rd((a & 0xFF00) | ((a + Y) & 0xFF)); return a + Y; }  // Exception - always dummy read.
inline u16 abx()   {
    u16 a = abs();
    if (cross(a, X)) {
        u16 dummy_addr = (a & 0xFF00) | ((a + X) & 0xFF);
        rd(dummy_addr);
    }
    return a + X;
}
inline u16 aby()   {
    u16 a = abs();
    if (cross(a, Y)) {
        u16 dummy_addr = (a & 0xFF00) | ((a + Y) & 0xFF);
        rd(dummy_addr);
    }
    return a + Y;
}
inline u16 zp()    { return rd(imm());                                  }
inline u16 zpx()   { T; return (zp() + X) % 0x100;                      }
inline u16 zpy()   { T; return (zp() + Y) % 0x100;                      }
inline u16 izx()   { u8 zp_addr = zp(); rd(zp_addr); u8 i = (zp_addr + X) % 0x100; return rd16_d(i, (i+1) % 0x100);     }
inline u16 _izy()  { u8 i = zp();  return rd16_d(i, (i+1) % 0x100) + Y; }  // Exception.
inline u16 izy()   { u16 a = _izy(); if (cross(a-Y, Y)) { rd(((a - Y) & 0xFF00) | (a & 0xFF)); } return a;    }

/* STx */
template<u8& r, Mode m> void st()        {    wr(   m()    , r); }
template<>              void st<A,izy>() { u16 a = _izy(); rd_dummy(((a - Y) & 0xFF00) | (a & 0xFF)); wr(a, A); }  // Always dummy read
template<>              void st<A,abx>() { u16 a = abs(); rd_dummy((a & 0xFF00) | ((a + X) & 0xFF)); wr(a + X, A); }  // Always dummy read
template<>              void st<A,aby>() { u16 a = abs(); rd_dummy((a & 0xFF00) | ((a + Y) & 0xFF)); wr(a + Y, A); }  // Always dummy read

#define G  u16 a = m(); u8 p = rd(a)  /* Fetch parameter */
template<u8& r, Mode m> void ld()  { G; upd_nz(r = p);                  }  // LDx
template<u8& r, Mode m> void cmp() { G; upd_nz(r - p); P[C] = (r >= p); }  // CMP, CPx
/* Arithmetic and bitwise */
template<Mode m> void ADC() { G       ; s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }
template<Mode m> void SBC() { G ^ 0xFF; s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }
template<Mode m> void BIT() { G; P[Z] = !(A & p); P[N] = p & 0x80; P[V] = p & 0x40; }
template<Mode m> void AND() { G; upd_nz(A &= p); }
template<Mode m> void EOR() { G; upd_nz(A ^= p); }
template<Mode m> void ORA() { G; upd_nz(A |= p); }
/* Read-Modify-Write */
template<Mode m> void ASL() { G; P[C] = p & 0x80; wr(a, p); is_rmw_cycle = true; upd_nz(wr(a, p << 1)); is_rmw_cycle = false; }
template<Mode m> void LSR() { G; P[C] = p & 0x01; wr(a, p); is_rmw_cycle = true; upd_nz(wr(a, p >> 1)); is_rmw_cycle = false; }
template<Mode m> void ROL() { G; u8 c = P[C]     ; P[C] = p & 0x80; wr(a, p); is_rmw_cycle = true; upd_nz(wr(a, (p << 1) | c) ); is_rmw_cycle = false; }
template<Mode m> void ROR() { G; u8 c = P[C] << 7; P[C] = p & 0x01; wr(a, p); is_rmw_cycle = true; upd_nz(wr(a, c | (p >> 1)) ); is_rmw_cycle = false; }
template<Mode m> void DEC() { G; wr(a, p); is_rmw_cycle = true; upd_nz(wr(a, --p)); is_rmw_cycle = false; }
template<Mode m> void INC() { G; wr(a, p); is_rmw_cycle = true; upd_nz(wr(a, ++p)); is_rmw_cycle = false; }
#undef G

/* DEx, INx */
template<u8& r> void dec() { rd(PC+1); upd_nz(--r); }  // Dummy read from PC+1
template<u8& r> void inc() { rd(PC+1); upd_nz(++r); }  // Dummy read from PC+1
/* Bit shifting on the accumulator */
void ASL_A() { P[C] = A & 0x80; upd_nz(A <<= 1); rd(PC+1); }  // Dummy read from PC+1
void LSR_A() { P[C] = A & 0x01; upd_nz(A >>= 1); rd(PC+1); }  // Dummy read from PC+1
void ROL_A() { u8 c = P[C]     ; P[C] = A & 0x80; upd_nz(A = ((A << 1) | c) ); rd(PC+1); }  // Dummy read from PC+1
void ROR_A() { u8 c = P[C] << 7; P[C] = A & 0x01; upd_nz(A = (c | (A >> 1)) ); rd(PC+1); }  // Dummy read from PC+1

/* Txx (move values between registers) */
template<u8& s, u8& d> void tr()      { rd(PC+1); upd_nz(d = s); }  // Dummy read from PC+1
template<>             void tr<X,S>() { rd(PC+1); S = X;         }  // TSX, exception.

/* Unofficial opcodes */
#define G  u16 a = m(); u8 p = rd(a)  /* Fetch parameter */

// SLO/ASO: ASL + ORA
template<Mode m> void SLO() { G; P[C] = p & 0x80; wr(a, p); p <<= 1; wr(a, p); upd_nz(A |= p); }
template<> void SLO<izx>() { u16 a = izx(); u8 p = rd(a); P[C] = p & 0x80; wr(a, p); p <<= 1; wr(a, p); upd_nz(A |= p); }
template<> void SLO<izy>() { u8 zp = rd(imm()); u16 base = rd16_d(zp, (zp + 1) % 0x100); u16 a = base + Y; if (!cross(base, Y)) T; else rd(((base) & 0xFF00) | (a & 0xFF)); u8 p = rd(a); P[C] = p & 0x80; wr(a, p); p <<= 1; wr(a, p); upd_nz(A |= p); }

// RLA: ROL + AND
template<Mode m> void RLA() { G; u8 c = P[C]; P[C] = p & 0x80; wr(a, p); p = (p << 1) | c; wr(a, p); upd_nz(A &= p); }
template<> void RLA<izx>() { u16 a = izx(); u8 p = rd(a); u8 c = P[C]; P[C] = p & 0x80; wr(a, p); p = (p << 1) | c; wr(a, p); upd_nz(A &= p); }
template<> void RLA<izy>() { u8 zp = rd(imm()); u16 base = rd16_d(zp, (zp + 1) % 0x100); u16 a = base + Y; if (!cross(base, Y)) T; else rd(((base) & 0xFF00) | (a & 0xFF)); u8 p = rd(a); u8 c = P[C]; P[C] = p & 0x80; wr(a, p); p = (p << 1) | c; wr(a, p); upd_nz(A &= p); }

// SRE/LSE: LSR + EOR
template<Mode m> void SRE() { G; P[C] = p & 0x01; wr(a, p); p >>= 1; wr(a, p); upd_nz(A ^= p); }
template<> void SRE<izx>() { u16 a = izx(); u8 p = rd(a); P[C] = p & 0x01; wr(a, p); p >>= 1; wr(a, p); upd_nz(A ^= p); }
template<> void SRE<izy>() { u8 zp = rd(imm()); u16 base = rd16_d(zp, (zp + 1) % 0x100); u16 a = base + Y; if (!cross(base, Y)) T; else rd(((base) & 0xFF00) | (a & 0xFF)); u8 p = rd(a); P[C] = p & 0x01; wr(a, p); p >>= 1; wr(a, p); upd_nz(A ^= p); }

// RRA: ROR + ADC
template<Mode m> void RRA() { G; u8 c = P[C] << 7; P[C] = p & 0x01; wr(a, p); p = c | (p >> 1); wr(a, p); s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }
template<> void RRA<izx>() { u16 a = izx(); u8 p = rd(a); u8 c = P[C] << 7; P[C] = p & 0x01; wr(a, p); p = c | (p >> 1); wr(a, p); s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }
template<> void RRA<izy>() { u8 zp = rd(imm()); u16 base = rd16_d(zp, (zp + 1) % 0x100); u16 a = base + Y; if (!cross(base, Y)) T; else rd(((base) & 0xFF00) | (a & 0xFF)); u8 p = rd(a); u8 c = P[C] << 7; P[C] = p & 0x01; wr(a, p); p = c | (p >> 1); wr(a, p); s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }

// DCP: DEC + CMP
template<Mode m> void DCP() { G; wr(a, p); --p; wr(a, p); upd_nz(A - p); P[C] = (A >= p); }
template<> void DCP<izx>() { u16 a = izx(); u8 p = rd(a); wr(a, p); --p; wr(a, p); upd_nz(A - p); P[C] = (A >= p); }
template<> void DCP<izy>() { u8 zp = rd(imm()); u16 base = rd16_d(zp, (zp + 1) % 0x100); u16 a = base + Y; if (!cross(base, Y)) T; else rd(((base) & 0xFF00) | (a & 0xFF)); u8 p = rd(a); wr(a, p); --p; wr(a, p); upd_nz(A - p); P[C] = (A >= p); }

// ISC/ISB: INC + SBC
template<Mode m> void ISC() { G; wr(a, p); ++p; wr(a, p); p ^= 0xFF; s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }
template<> void ISC<izx>() { u16 a = izx(); u8 p = rd(a); wr(a, p); ++p; wr(a, p); p ^= 0xFF; s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }
template<> void ISC<izy>() { u8 zp = rd(imm()); u16 base = rd16_d(zp, (zp + 1) % 0x100); u16 a = base + Y; if (!cross(base, Y)) T; else rd(((base) & 0xFF00) | (a & 0xFF)); u8 p = rd(a); wr(a, p); ++p; wr(a, p); p ^= 0xFF; s16 r = A + p + P[C]; upd_cv(A, p, r); upd_nz(A = r); }

// SAX: Store A & X
template<Mode m> void SAX() { wr(m(), A & X); }
template<> void SAX<izx>() { u16 a = izx(); wr(a, A & X); }

// LAX: LDA + LDX
template<Mode m> void LAX() { G; upd_nz(A = X = p); }
template<> void LAX<izx>() { u16 a = izx(); u8 p = rd(a); upd_nz(A = X = p); }
template<> void LAX<izy>() { u16 a = izy(); u8 p = rd(a); upd_nz(A = X = p); }

#undef G

// Special unofficial NOPs with different sizes/timings
void NOP_imm() { rd(imm()); }         // 2-byte NOP (2 cycles)
void NOP_zp()  { rd(zp()); }          // 2-byte NOP with zp read (3 cycles)
void NOP_zpx() { rd(zpx()); }         // 2-byte NOP with zpx read (4 cycles)
void NOP_abs() { rd(abs()); }         // 3-byte NOP (4 cycles)
void NOP_abx() { rd(abx()); }         // 3-byte NOP (page cross handled in abx)

// ANC: AND with immediate, copy N to C
void ANC() { u8 p = rd(imm()); upd_nz(A &= p); P[C] = P[N]; }

// ALR: AND with immediate, then LSR
void ALR() { u8 p = rd(imm()); A &= p; P[C] = A & 0x01; upd_nz(A >>= 1); }

// ARR: AND with immediate, then ROR
void ARR() { u8 p = rd(imm()); A &= p; A = (P[C] << 7) | (A >> 1); P[C] = (A & 0x40) >> 6; P[V] = ((A & 0x40) >> 6) ^ ((A & 0x20) >> 5); upd_nz(A); }

// XAA (unstable): TXA + AND immediate
void XAA() { A = X; u8 p = rd(imm()); upd_nz(A &= p); }

// LAX immediate: LDA immediate + LDX immediate
void LAX_imm() { u8 p = rd(imm()); upd_nz(A = X = p); }

// AXS/SBX: X = (A & X) - immediate
void AXS() { u8 p = rd(imm()); u8 temp = A & X; X = temp - p; P[C] = (temp >= p); upd_nz(X); }

// SHA/AHX: Store A & X & (high byte + 1) - Complex behavior (using behavior 2)
void SHA_izy() {
    u8 zp = rd(imm());
    u16 base = rd16_d(zp, (zp + 1) % 0x100);
    u16 addr = base + Y;
    u8 h = (base >> 8) + 1;  // High byte of BASE address + 1

    // Always do dummy read (makes it always 6 cycles)
    rd((base & 0xFF00) | (addr & 0xFF));

    if (cross(base, Y)) {
        // Behavior 2: high byte of result ANDs with X only
        u8 addr_high = (addr >> 8) & X;
        addr = (addr & 0xFF) | (addr_high << 8);
    }

    wr(addr, A & X & h);
}

void SHA_aby() {
    u16 base = abs();
    u16 addr = base + Y;
    u8 h = (base >> 8) + 1;  // High byte of BASE address + 1

    // Always perform dummy read (these instructions always take 5 cycles)
    rd((base & 0xFF00) | ((base + Y) & 0xFF));

    // Page crossing behavior for absolute,Y
    if (cross(base, Y)) {
        // Behavior 2: high byte of result ANDs with X only
        u8 addr_high = (addr >> 8) & X;
        addr = (addr & 0xFF) | (addr_high << 8);
    }

    wr(addr, A & X & h);
}

// SHY: Store Y & (high byte + 1)
void SHY() {
    u16 base = abs();
    u16 addr = base + X;
    u8 h_plus_1 = ((base >> 8) + 1) & 0xFF;  // High byte of BASE address + 1

    // Always perform dummy read (these instructions always take 5 cycles)
    rd((base & 0xFF00) | ((base + X) & 0xFF));

    if (cross(base, X)) {
        // Page crossed: high byte gets corrupted
        // The corrupted high byte is: Y & (H+1)
        u8 addr_high = Y & h_plus_1;
        addr = (addr & 0xFF) | (addr_high << 8);
    }

    // Always store Y & (H+1)
    wr(addr, Y & h_plus_1);
}

// SHX: Store X & (high byte + 1)
void SHX() {
    u16 base = abs();
    u16 addr = base + Y;
    u8 h_plus_1 = ((base >> 8) + 1) & 0xFF;  // High byte of BASE address + 1

    // Always perform dummy read (these instructions always take 5 cycles)
    rd((base & 0xFF00) | ((base + Y) & 0xFF));

    if (cross(base, Y)) {
        // Page crossed: high byte gets corrupted
        // The corrupted high byte is: X & (H+1)
        u8 addr_high = X & h_plus_1;
        addr = (addr & 0xFF) | (addr_high << 8);
    }

    // Always store X & (H+1)
    wr(addr, X & h_plus_1);
}

// TAS/SHS: Transfer A & X to S, store A & X & (high byte + 1)
void TAS() {
    S = A & X;
    u16 base = abs();
    u16 addr = base + Y;
    u8 h_plus_1 = ((base >> 8) + 1) & 0xFF;  // High byte of BASE address + 1

    // Always perform dummy read (these instructions always take 5 cycles)
    rd((base & 0xFF00) | ((base + Y) & 0xFF));

    if (cross(base, Y)) {
        // Page crossed: high byte of address is incremented (already done in addr calculation)
        // then ANDed with (A & X)
        u8 addr_high = (addr >> 8) & S;  // S = A & X
        addr = (addr & 0xFF) | (addr_high << 8);
    }

    // Always store (A & X) & (H+1)
    wr(addr, S & h_plus_1);
}

// LAS: Load A, X, S with memory & S
void LAS() { u16 a = aby(); u8 p = rd(a); S &= p; upd_nz(A = X = S); }

/* Stack operations */
void PLP() {
    // Cycle 2: Dummy read from PC+1 (polling already happened in exec() before I flag changes)
    rd(PC+1);
    T;  // Cycle 3: Dummy read at stack pointer
    P.set(pop());  // Cycle 4: Pull flags from stack
}
void PHP() { rd(PC+1); push(P.get() | (1 << 4)); }  // Dummy read from PC+1, B flag set
void PLA() { rd(PC+1); T; A = pop(); upd_nz(A);  }  // Dummy read from PC+1, then stack operation
void PHA() { rd(PC+1); push(A); }  // Dummy read from PC+1

/* Forward declarations for interrupt handling */
template<IntType t> void INT();

/* Flow control (branches, jumps) */
template<Flag f, bool v> void br()
{
    s8 j = rd(imm());  // Cycle 2: Read operand (polling already happened in exec())
    if (P[f] == v) {
        if (cross(PC, j)) {
            rd((PC & 0xFF00) | ((PC + j) & 0xFF));  // Cycle 3: Dummy read on page cross
            // Poll before cycle 4 for page-crossing branches
            if (nmi_latch) {
                nmi_latch = false;  // Clear latch when servicing NMI
                INT<NMI>();
                return;
            }
            else if (irq && !P[I]) { INT<IRQ>(); return; }
        }
        T; PC += j;  // Cycle 3 (no cross) or 4 (page cross): Update PC
    }
}
void JMP_IND() { u16 i = rd16(imm16()); PC = rd16_d(i, (i&0xFF00) | ((i+1) % 0x100)); }
void JMP()     { PC = rd16(imm16()); }
void JSR()     { u16 t = PC+1; T; push(t >> 8); push(t); PC = rd16(imm16()); }

/* Return instructions */
void RTS() { rd(PC+1); T;  PC = (pop() | (pop() << 8)) + 1; T; }  // Dummy read from PC+1
void RTI() {
    interrupt_already_polled = true;  // Skip general poll in exec()
    rd(PC+1);  // Cycle 2: Dummy read from PC+1
    T;         // Cycle 3: Dummy read at stack pointer
    P.set(pop());  // Cycle 4: Pull flags from stack (I flag updated)
    PC = pop() | (pop() << 8);  // Cycles 5-6: Pull PC from stack

    // Poll for interrupts AFTER restoring PC and flags
    // The restored PC is the correct value to push if interrupt occurs
    if (nmi_latch) {
        nmi_latch = false;  // Clear latch when servicing NMI
        INT<NMI>();
    }
    else if (irq && !P[I]) INT<IRQ>();
}

template<Flag f, bool v> void flag() {
    // Cycle 2: Dummy read from PC+1 (polling already happened in exec())
    rd(PC+1);
    // Modify the flag after polling, so CLI/SEI poll before the I flag changes
    P[f] = v;
}
template<IntType t> void INT()
{
    if (t == BRK) rd(PC+1);  // BRK performs dummy read from PC+1
    else T;
    if (t != BRK) T;  // Non-BRK interrupts have additional cycle
    if (t == BRK) PC++;  // BRK increments PC before pushing
    if (t != RESET)  // Writes on stack are inhibited on RESET.
    {
        push(PC >> 8); push(PC & 0xFF);
        push(P.get() | ((t == BRK) << 4));  // Set B if BRK.
    }
    else { S -= 3; T; T; T; }
    P[I] = true;
                          /*   NMI    Reset    IRQ     BRK  */
    constexpr u16 vect[] = { 0xFFFA, 0xFFFC, 0xFFFE, 0xFFFE };

    // Vector fetch with NMI hijacking check
    u16 vector_addr = vect[t];

    // Check for NMI hijacking BEFORE vector fetch
    // This applies to BRK and IRQ (not NMI itself or RESET)
    if ((t == BRK || t == IRQ) && nmi_latch) {
        // NMI hijacks the interrupt - use NMI vector instead
        vector_addr = 0xFFFA;
        nmi_latch = false;  // Clear NMI latch
    }

    // Cycles 6-7: Read interrupt vector
    PC = rd16(vector_addr);

    // For normal NMI, latch is already cleared in polling code
}
void NOP() { rd(PC+1); }  // Dummy read from PC+1

/* Execute a CPU instruction */
void exec()
{
    u8 opcode = rd(PC++);  // Cycle 1: Fetch the opcode (PC now points to next byte)

    // General interrupt polling: happens after cycle 1 (reading opcode), before cycle 2.
    // Special instructions (RTI) may override this by setting interrupt_already_polled.
    if (!interrupt_already_polled) {
        if (nmi_latch) {
            PC--;  // Point back to the thrown-away opcode
            nmi_latch = false;  // Clear latch when servicing NMI
            INT<NMI>();
            return;
        }
        else if (irq && !P[I]) {
            PC--;  // Point back to the thrown-away opcode
            INT<IRQ>();
            return;
        }
    }
    interrupt_already_polled = false;  // Reset for next instruction

    // Promote pending NMI to latch (provides 1-instruction delay for interrupt latency)
    if (nmi_pending) {
        nmi_latch = true;
        nmi_pending = false;
    }

    switch (opcode)
    {
        // Select the right function to emulate the instruction:
        case 0x00: return INT<BRK>()  ;  case 0x01: return ORA<izx>()  ;
        case 0x03: return SLO<izx>()  ;  case 0x04: return NOP_zp()    ;
        case 0x05: return ORA<zp>()   ;  case 0x06: return ASL<zp>()   ;
        case 0x07: return SLO<zp>()   ;
        case 0x08: return PHP()       ;  case 0x09: return ORA<imm>()  ;
        case 0x0A: return ASL_A()     ;  case 0x0B: return ANC()       ;
        case 0x0C: return NOP_abs()   ;  case 0x0D: return ORA<abs>()  ;
        case 0x0E: return ASL<abs>()  ;  case 0x0F: return SLO<abs>()  ;
        case 0x10: return br<N,0>()   ;
        case 0x11: return ORA<izy>()  ;  case 0x13: return SLO<izy>()  ;
        case 0x14: return NOP_zpx()   ;  case 0x15: return ORA<zpx>()  ;
        case 0x16: return ASL<zpx>()  ;  case 0x17: return SLO<zpx>()  ;
        case 0x18: return flag<C,0>() ;
        case 0x19: return ORA<aby>()  ;  case 0x1A: return NOP()       ;
        case 0x1B: return SLO<_aby>() ;
        case 0x1C: return NOP_abx()   ;  case 0x1D: return ORA<abx>()  ;
        case 0x1E: return ASL<_abx>() ;  case 0x1F: return SLO<_abx>() ;
        case 0x20: return JSR()       ;
        case 0x21: return AND<izx>()  ;  case 0x23: return RLA<izx>()  ;
        case 0x24: return BIT<zp>()   ;
        case 0x25: return AND<zp>()   ;  case 0x26: return ROL<zp>()   ;
        case 0x27: return RLA<zp>()   ;
        case 0x28: return PLP()       ;  case 0x29: return AND<imm>()  ;
        case 0x2A: return ROL_A()     ;  case 0x2B: return ANC()       ;
        case 0x2C: return BIT<abs>()  ;
        case 0x2D: return AND<abs>()  ;  case 0x2E: return ROL<abs>()  ;
        case 0x2F: return RLA<abs>()  ;
        case 0x30: return br<N,1>()   ;  case 0x31: return AND<izy>()  ;
        case 0x33: return RLA<izy>()  ;  case 0x34: return NOP_zpx()   ;
        case 0x35: return AND<zpx>()  ;  case 0x36: return ROL<zpx>()  ;
        case 0x37: return RLA<zpx>()  ;
        case 0x38: return flag<C,1>() ;  case 0x39: return AND<aby>()  ;
        case 0x3A: return NOP()       ;
        case 0x3B: return RLA<_aby>() ;  case 0x3C: return NOP_abx()   ;
        case 0x3D: return AND<abx>()  ;  case 0x3E: return ROL<_abx>() ;
        case 0x3F: return RLA<_abx>() ;
        case 0x40: return RTI()       ;  case 0x41: return EOR<izx>()  ;
        case 0x43: return SRE<izx>()  ;  case 0x44: return NOP_zp()    ;
        case 0x45: return EOR<zp>()   ;  case 0x46: return LSR<zp>()   ;
        case 0x47: return SRE<zp>()   ;
        case 0x48: return PHA()       ;  case 0x49: return EOR<imm>()  ;
        case 0x4A: return LSR_A()     ;  case 0x4B: return ALR()       ;
        case 0x4C: return JMP()       ;
        case 0x4D: return EOR<abs>()  ;  case 0x4E: return LSR<abs>()  ;
        case 0x4F: return SRE<abs>()  ;
        case 0x50: return br<V,0>()   ;  case 0x51: return EOR<izy>()  ;
        case 0x53: return SRE<izy>()  ;  case 0x54: return NOP_zpx()   ;
        case 0x55: return EOR<zpx>()  ;  case 0x56: return LSR<zpx>()  ;
        case 0x57: return SRE<zpx>()  ;
        case 0x58: return flag<I,0>() ;  case 0x59: return EOR<aby>()  ;
        case 0x5A: return NOP()       ;
        case 0x5B: return SRE<_aby>() ;  case 0x5C: return NOP_abx()   ;
        case 0x5D: return EOR<abx>()  ;  case 0x5E: return LSR<_abx>() ;
        case 0x5F: return SRE<_abx>() ;
        case 0x60: return RTS()       ;  case 0x61: return ADC<izx>()  ;
        case 0x63: return RRA<izx>()  ;  case 0x64: return NOP_zp()    ;
        case 0x65: return ADC<zp>()   ;  case 0x66: return ROR<zp>()   ;
        case 0x67: return RRA<zp>()   ;
        case 0x68: return PLA()       ;  case 0x69: return ADC<imm>()  ;
        case 0x6A: return ROR_A()     ;  case 0x6B: return ARR()       ;
        case 0x6C: return JMP_IND()   ;
        case 0x6D: return ADC<abs>()  ;  case 0x6E: return ROR<abs>()  ;
        case 0x6F: return RRA<abs>()  ;
        case 0x70: return br<V,1>()   ;  case 0x71: return ADC<izy>()  ;
        case 0x73: return RRA<izy>()  ;  case 0x74: return NOP_zpx()   ;
        case 0x75: return ADC<zpx>()  ;  case 0x76: return ROR<zpx>()  ;
        case 0x77: return RRA<zpx>()  ;
        case 0x78: return flag<I,1>() ;  case 0x79: return ADC<aby>()  ;
        case 0x7A: return NOP()       ;
        case 0x7B: return RRA<_aby>() ;  case 0x7C: return NOP_abx()   ;
        case 0x7D: return ADC<abx>()  ;  case 0x7E: return ROR<_abx>() ;
        case 0x7F: return RRA<_abx>() ;  case 0x80: return NOP_imm()   ;
        case 0x81: return st<A,izx>() ;  case 0x82: return NOP_imm()   ;
        case 0x83: return SAX<izx>()  ;  case 0x84: return st<Y,zp>()  ;
        case 0x85: return st<A,zp>()  ;  case 0x86: return st<X,zp>()  ;
        case 0x87: return SAX<zp>()   ;  case 0x89: return NOP_imm()   ;
        case 0x88: return dec<Y>()    ;  case 0x8A: return tr<X,A>()   ;
        case 0x8B: return XAA()       ;
        case 0x8C: return st<Y,abs>() ;  case 0x8D: return st<A,abs>() ;
        case 0x8E: return st<X,abs>() ;  case 0x8F: return SAX<abs>()  ;
        case 0x90: return br<C,0>()   ;
        case 0x91: return st<A,izy>() ;  case 0x93: return SHA_izy()   ;
        case 0x94: return st<Y,zpx>() ;
        case 0x95: return st<A,zpx>() ;  case 0x96: return st<X,zpy>() ;
        case 0x97: return SAX<zpy>()  ;
        case 0x98: return tr<Y,A>()   ;  case 0x99: return st<A,aby>() ;
        case 0x9B: return TAS()       ;  case 0x9C: return SHY()       ;
        case 0x9A: return tr<X,S>()   ;  case 0x9D: return st<A,abx>() ;
        case 0x9E: return SHX()       ;  case 0x9F: return SHA_aby()   ;
        case 0xA0: return ld<Y,imm>() ;  case 0xA1: return ld<A,izx>() ;
        case 0xA2: return ld<X,imm>() ;  case 0xA3: return LAX<izx>()  ;
        case 0xA4: return ld<Y,zp>()  ;
        case 0xA5: return ld<A,zp>()  ;  case 0xA6: return ld<X,zp>()  ;
        case 0xA7: return LAX<zp>()   ;
        case 0xA8: return tr<A,Y>()   ;  case 0xA9: return ld<A,imm>() ;
        case 0xAA: return tr<A,X>()   ;  case 0xAB: return LAX_imm()   ;
        case 0xAC: return ld<Y,abs>() ;
        case 0xAD: return ld<A,abs>() ;  case 0xAE: return ld<X,abs>() ;
        case 0xAF: return LAX<abs>()  ;
        case 0xB0: return br<C,1>()   ;  case 0xB1: return ld<A,izy>() ;
        case 0xB3: return LAX<izy>()  ;
        case 0xB4: return ld<Y,zpx>() ;  case 0xB5: return ld<A,zpx>() ;
        case 0xB6: return ld<X,zpy>() ;  case 0xB7: return LAX<zpy>()  ;
        case 0xB8: return flag<V,0>() ;
        case 0xB9: return ld<A,aby>() ;  case 0xBA: return tr<S,X>()   ;
        case 0xBB: return LAS()       ;
        case 0xBC: return ld<Y,abx>() ;  case 0xBD: return ld<A,abx>() ;
        case 0xBE: return ld<X,aby>() ;  case 0xBF: return LAX<aby>()  ;
        case 0xC0: return cmp<Y,imm>();
        case 0xC1: return cmp<A,izx>();  case 0xC2: return NOP_imm()   ;
        case 0xC3: return DCP<izx>()  ;  case 0xC4: return cmp<Y,zp>() ;
        case 0xC5: return cmp<A,zp>() ;  case 0xC6: return DEC<zp>()   ;
        case 0xC7: return DCP<zp>()   ;
        case 0xC8: return inc<Y>()    ;  case 0xC9: return cmp<A,imm>();
        case 0xCA: return dec<X>()    ;  case 0xCB: return AXS()       ;
        case 0xCC: return cmp<Y,abs>();
        case 0xCD: return cmp<A,abs>();  case 0xCE: return DEC<abs>()  ;
        case 0xCF: return DCP<abs>()  ;
        case 0xD0: return br<Z,0>()   ;  case 0xD1: return cmp<A,izy>();
        case 0xD3: return DCP<izy>()  ;  case 0xD4: return NOP_zpx()   ;
        case 0xD5: return cmp<A,zpx>();  case 0xD6: return DEC<zpx>()  ;
        case 0xD7: return DCP<zpx>()  ;
        case 0xD8: return flag<D,0>() ;  case 0xD9: return cmp<A,aby>();
        case 0xDA: return NOP()       ;  case 0xDB: return DCP<_aby>() ;
        case 0xDC: return NOP_abx()   ;
        case 0xDD: return cmp<A,abx>();  case 0xDE: return DEC<_abx>() ;
        case 0xDF: return DCP<_abx>() ;
        case 0xE0: return cmp<X,imm>();  case 0xE1: return SBC<izx>()  ;
        case 0xE2: return NOP_imm()   ;  case 0xE3: return ISC<izx>()  ;
        case 0xE4: return cmp<X,zp>() ;  case 0xE5: return SBC<zp>()   ;
        case 0xE6: return INC<zp>()   ;  case 0xE7: return ISC<zp>()   ;
        case 0xE8: return inc<X>()    ;
        case 0xE9: return SBC<imm>()  ;  case 0xEA: return NOP()       ;
        case 0xEB: return SBC<imm>()  ;
        case 0xEC: return cmp<X,abs>();  case 0xED: return SBC<abs>()  ;
        case 0xEE: return INC<abs>()  ;  case 0xEF: return ISC<abs>()  ;
        case 0xF0: return br<Z,1>()   ;
        case 0xF1: return SBC<izy>()  ;  case 0xF3: return ISC<izy>()  ;
        case 0xF4: return NOP_zpx()   ;  case 0xF5: return SBC<zpx>()  ;
        case 0xF6: return INC<zpx>()  ;  case 0xF7: return ISC<zpx>()  ;
        case 0xF8: return flag<D,1>() ;
        case 0xF9: return SBC<aby>()  ;  case 0xFA: return NOP()       ;
        case 0xFB: return ISC<_aby>() ;  case 0xFC: return NOP_abx()   ;
        case 0xFD: return SBC<abx>()  ;
        case 0xFE: return INC<_abx>() ;  case 0xFF: return ISC<_abx>() ;
        default:
            std::cout << "Invalid Opcode! PC: " << PC << " Opcode: 0x" << std::hex << (int)(rd(PC - 1)) << "\n";
            return NOP();
    }
}

void set_nmi(bool v) {
    // Edge detection: detect 0→1 transition
    if (!nmi_previous && v) {
        nmi_pending = true;  // Rising edge detected, set pending (will be latched after 1 instruction)
    }
    nmi_previous = v;
    nmi_line = v;
}

void set_irq(bool v) { irq = v; }

int dmc_read(void*, cpu_addr_t addr) { return access<0>(addr); }

/* Turn on the CPU */
void power()
{
    remainingCycles = 0;

    P.set(0x04);
    A = X = Y = S = 0x00;
    memset(ram, 0xFF, sizeof(ram));

    nmi_line = nmi_pending = nmi_latch = nmi_previous = false;
    irq = false;
    INT<RESET>();
}

/* Run the CPU for roughly a frame */
void run_frame()
{
    remainingCycles += TOTAL_CYCLES;

    while (remainingCycles > 0)
    {
        // Interrupt polling now happens in exec() after reading the opcode,
        // with special handling for RTI which polls after restoring flags.
        exec();
    }

    APU::run_frame(elapsed());
}


}
