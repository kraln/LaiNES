# Test 84 Hang Analysis: "NMI Overlap BRK" (CORRECTED)

## The Test
Test #84 is "NMI Overlap BRK" which tests interrupt hijacking - what happens when an NMI interrupt occurs during BRK instruction execution.

## Test Setup (Correct Understanding)
The ROM correctly sets up interrupt vectors at $FFFA-$FFFF:
- **$FFFA-$FFFB**: Points to $0700 (NMI handler)
- **$FFFE-$FFFF**: Points to $0600 (BRK/IRQ handler)

At runtime, the test sets up handlers at these locations:
- **$600**: `ORA #$80` followed by `JMP TEST_NmiAndBrk_BRK`
- **$700**: `ORA #$80` followed by `JMP TEST_NmiAndBrk_NMI`

The `ORA #$80` instruction is intentional - it sets bit 7 of the accumulator as a marker to detect which handler was called.

## What the Test Does
1. Synchronizes timing to trigger NMI at precise cycles relative to BRK
2. Executes BRK instruction: 
   ```asm
   BRK  ; Should skip the next instruction
   INY  ; This should be skipped!
   ```
3. Tests various timing scenarios where NMI occurs during different cycles of BRK execution
4. Stores processor flags from each interrupt to verify correct behavior

## The Real Bug in LaiNES

### How LaiNES Handles Interrupts (WRONG)
```cpp
while (remainingCycles > 0)
{
    if (nmi) INT<NMI>();           // Check interrupts BEFORE instruction
    else if (irq and !P[I]) INT<IRQ>();
    
    exec();                         // Then execute instruction
}
```

### The Problem: No Interrupt Hijacking Support
1. **LaiNES checks interrupts only BEFORE each instruction**
2. **Once BRK starts executing, it can't be hijacked by NMI**
3. **Real 6502 checks for NMI during specific cycles of BRK execution**

### What Should Happen (Interrupt Hijacking)
On real hardware:
1. BRK instruction begins execution
2. During BRK's execution cycles, NMI line goes active
3. When BRK reaches the cycle where it reads the interrupt vector:
   - If NMI is pending, it reads from $FFFA-$FFFB (NMI vector) instead of $FFFE-$FFFF
   - This is called "interrupt hijacking"
4. CPU jumps to NMI handler ($700) even though BRK was executing

### Why It Hangs
The test is carefully timed so NMI occurs during BRK execution. In LaiNES:
1. BRK starts executing (NMI wasn't pending at instruction start)
2. NMI becomes pending during BRK execution
3. BRK completes normally, jumping to $600
4. The test expects NMI hijacking to have occurred
5. The test fails its internal checks and likely enters an invalid state
6. **The hang occurs because the test's state machine is broken** - it expected different interrupt behavior

## The Fix Needed
LaiNES needs to implement interrupt hijacking:
1. Check for NMI during BRK execution, not just before
2. If NMI is pending when BRK reads the vector, use NMI vector instead
3. This requires cycle-accurate interrupt polling within the INT<BRK>() function

## Why Other Tests Pass
- Earlier interrupt tests don't test hijacking scenarios
- They test simple cases where interrupts occur between instructions
- Test 84 specifically tests this complex edge case that LaiNES doesn't handle