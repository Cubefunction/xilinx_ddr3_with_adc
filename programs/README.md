# Host scripts for the FPGA project

`send_adc_test.py` is the one entry point — it talks to `uart_top.sv` over
UART and can do three things:

1. Boot the AD9833 DDS chip into a known sine output
2. Drive the on-chip ADC core with a small "program" (NOP / SAM / JMP / END)
3. Stream the captured DDR3 region back through `reg[58]` (STREAM)

This README covers the host-side details for (1) and (2).  Everything else
(write/read protocol, register map) is in the docstring at the top of
`send_adc_test.py`.

---

## 1. AD9833 control (`reg[0..3]`)

The four UART registers below feed straight into the AD9833 sub-block.
At a rising edge on `reg[3][0]`, the FSM latches the 5 SPI words once and
fires them out on the SPI bus to the chip:

| UART reg | name        | mapping                                 |
|----------|-------------|-----------------------------------------|
| `reg[0]` | cmd         | low 16 b -> SPI word 0 (control)        |
| `reg[1]` | freq        | low 16 b -> word 1; high 16 b -> word 2 |
| `reg[2]` | phase_ctrl  | low 16 b -> word 3; high 16 b -> word 4 |
| `reg[3]` | control     | bit 0 = start (0 -> 1 rising edge)      |

Important: words 0..4 are **only** sampled at the moment `reg[3][0]`
goes high.  To re-program the chip after changing frequency you must
toggle `reg[3]`: write 0 first, then 1.  Holding it at 1 doesn't keep
re-firing.

### Default boot sequence

`configure_ad9833()` in `send_adc_test.py` runs this fixed init:

```python
write_reg(ser, 0, 0x0000_2100)   # cmd        : SPI word 0 = 0x2100
write_reg(ser, 1, 0x48D1_4567)   # freq       : word2=0x48D1, word1=0x4567
write_reg(ser, 2, 0x2000_C100)   # phase_ctrl : word4=0x2000, word3=0xC100
write_reg(ser, 3, 0x0000_0001)   # control    : start = 1  (rising edge)
```

What each SPI word does, with values worked out for the AD9833 dev-kit
(MCLK = 25 MHz):

| word | value    | role                                                                |
|------|----------|---------------------------------------------------------------------|
|  0   | `0x2100` | control: B28=1 (28-bit freq in two halves), RESET=1 (output muted)  |
|  1   | `0x4567` | FREQ0 register, low  14 b = `0x0567` = 1383                         |
|  2   | `0x48D1` | FREQ0 register, high 14 b = `0x08D1` = 2257                         |
|  3   | `0xC100` | PHASE0 register, 12 b = `0x100` = 256                               |
|  4   | `0x2000` | control: B28=1, RESET=0 -> release reset, begin sine output         |

Working out the actual output:

```
freq_reg  = (2257 << 14) | 1383 = 36_980_071
f_out     = freq_reg * MCLK / 2^28
          = 36_980_071 * 25_000_000 / 268_435_456
          ≈ 3.444 MHz                              <-- default sine frequency
phase_deg = 256 / 4096 * 360 ≈ 22.5°               <-- default PHASE0
waveform  = sine (OPBITEN = 0, MODE = 0)
```

Frequency resolution at 25 MHz MCLK is `25 MHz / 2^28 ≈ 0.0931 Hz`, so
any target between roughly 0 and 12.5 MHz is reachable.

### Skipping it

If your board already has the AD9833 configured, or you don't have one
populated, pass `--no-ad9833` to skip the boot sequence:

```powershell
python send_adc_test.py --no-ad9833 --psm programs\loop_4x32.psm
```

### Changing frequency / phase from Python

The 16-bit SPI words follow the AD9833 protocol:

* **FREQ0** = 28 bits, written as two 16-bit halves; each half has the
  top 2 bits = `01` (the register address) and the low 14 bits = data.
* **PHASE0** = 12 bits, top 4 bits = `1100` (= `0xC000` mask).
* **f_out** = `freq_reg * f_mclk / 2^28`  (with the on-board crystal,
  typically 25 MHz).

A handy snippet to reprogram frequency / phase without restarting the
script (works with the dev-kit's 25 MHz crystal):

```python
AD9833_MCLK_HZ = 25_000_000

def ad9833_set_freq_hz(ser, f_hz, phase_deg=0.0, mclk_hz=AD9833_MCLK_HZ):
    # 28-bit frequency word (truncated, fine at typical resolutions)
    freq_reg = int(round(f_hz * (1 << 28) / mclk_hz)) & 0x0FFF_FFFF
    lsb14    = (freq_reg      ) & 0x3FFF
    msb14    = (freq_reg >> 14) & 0x3FFF
    word1    = 0x4000 | lsb14                       # FREQ0 LSB
    word2    = 0x4000 | msb14                       # FREQ0 MSB

    # 12-bit phase word
    phase_reg = int(round(phase_deg / 360.0 * 4096)) & 0x0FFF
    word3     = 0xC000 | phase_reg                  # PHASE0
    word4     = 0x2000                              # control: B28=1, RESET=0

    write_reg(ser, 0, 0x0000_2100)                  # cmd: B28=1, hold RESET
    write_reg(ser, 1, (word2 << 16) | word1)        # FREQ0 (LSB | MSB)
    write_reg(ser, 2, (word4 << 16) | word3)        # PHASE0 + release RESET
    write_reg(ser, 3, 0)                            # drop start
    write_reg(ser, 3, 1)                            # rising edge -> SPI burst

# Examples:
#   ad9833_set_freq_hz(ser, 1_000_000)        # clean 1 MHz sine
#   ad9833_set_freq_hz(ser, 2_500_000, 90)    # 2.5 MHz, 90° phase
```

To stop the output: just write a reset cmd and retrigger:

```python
write_reg(ser, 0, 0x0000_2100)   # B28=1, RESET=1  (mute output)
write_reg(ser, 3, 0)
write_reg(ser, 3, 1)
```

---

## 2. ADC programs (.psm files)

Tiny text-format assembly for the in-FPGA ADC core.  Loaded and sent to
the FPGA via:

    python send_adc_test.py --no-ad9833 --psm programs/your_file.psm

### Syntax

```
; comments after  ;  #  or  //
label_name:                   ; label on its own line
label_name: nop 3             ; label + insn on the same line
nop  <delay>                  ; 0..65535 cycles of idle
sam  <count>                  ; capture <count> samples (0..4095)
jmp  <target> [count]         ; target = label name OR raw PC integer
                              ; count default = 0  (loop forever)
end
```

Numbers may be decimal (`12`), hex (`0x10`), or binary (`0b1010`).
Opcodes are case-insensitive.  Whitespace and commas are flexible.

### Hardware quick facts

| Field         | Width | Notes                                            |
|---------------|-------|--------------------------------------------------|
| opcode        | 4 b   | NOP=0x0, SAM=0x1, JMP=0x2, END=0xF              |
| count_field   | 12 b  | SAM count, or JMP loop count (0 = forever)      |
| delay/target  | 16 b  | NOP delay cycles; JMP target uses low 6 bits    |

The ADC core has 53 program slots (`reg[4]..reg[56]`); `reg[57]` is the
control / start register.  The assembler errors out if a program is
longer than 53 instructions.

### Tip

Use `--dry-run` to see exactly what bits will be sent, without opening
the serial port:

    python send_adc_test.py --psm programs/loop_4x32.psm --dry-run

---

## 3. Full register map (host view)

| reg     | direction | purpose                                       |
|---------|-----------|-----------------------------------------------|
|  0..3   | W         | AD9833 control (see section 1)                |
|  4..56  | W         | ADC program slots                             |
|   57    | W         | ADC control: write `0x8000_0000` to start;    |
|         |           | write `0` to park the core                    |
|   58    | R         | STREAM trigger; reading kicks `ddr3_reader`   |
|         |           | which then streams `bytes_written` bytes back |
|   59    | R         | Status overlay: `{3'b0, bytes_written[28:0]}` |
