"""
send_adc_test.py

Board-side companion to ddr_combine_tb.sv.  Drives uart_top on the FPGA over
the UART line and runs the same NOP / SAM / END test the TB does, then pulls
the written DDR3 region back through the STREAM register and verifies its
internal shape (right byte count, right zero-padding).

Protocol recap (must match uart_regs + uart_top):
    write(idx, d32) :  [ {1'b0, idx[6:0]} , d[31:24], d[23:16], d[15:8], d[7:0] ]
    read (idx)      :  [ {1'b1, idx[6:0]} ]  ->  FPGA sends 4 bytes MSB-first
    stream read     :  read(STREAM_REG=58) kicks ddr3_reader; FPGA then emits
                       `bytes_written` bytes back-to-back (MSB-first, 16 bytes
                       per 128-bit DDR3 word)

Register map (matches uart_top.sv defaults):
    reg[0..3]   AD9833
    reg[4..57]  ADC core         (reg[57] = ADC ctrl)
    reg[58]     STREAM trigger   (read -> stream the whole written region)
    reg[59]     STATUS overlay   (read -> {3'b0, bytes_written[28:0]})

Run:
    pip install pyserial
    python send_adc_test.py
"""

import argparse
import os
import re
import struct
import sys
import time

import serial


# ----------------------------------------------------------------------------
# Register-map constants (keep in sync with uart_top.sv parameters)
# ----------------------------------------------------------------------------
AD9833_BASE       = 0
ADC_CORE_BASE     = 4
ADC_CTRL_REG_IDX  = 53
ADC_CTRL_ADDR     = ADC_CORE_BASE + ADC_CTRL_REG_IDX    # 57
STREAM_REG        = 58
STATUS_REG        = 59

# DDR3 / ADC datapath constants
ADC_DATA_W        = 16
DDR_DATA_W        = 128
BYTES_PER_WORD    = DDR_DATA_W // 8                     # 16
P_WR_BURST_LEN    = 8
P_WR_BURST_NUM    = 1
WORDS_PER_TX      = P_WR_BURST_LEN * P_WR_BURST_NUM     # 8
BYTES_PER_TX      = WORDS_PER_TX * BYTES_PER_WORD       # 128
SAMPLES_PER_WORD  = DDR_DATA_W // ADC_DATA_W            # 8
SAMPLES_PER_TX    = SAMPLES_PER_WORD * WORDS_PER_TX     # 64

# ADC ctrl reg bit layout (matches adc_core)
ADC_CTRL_START    = 1 << 31                             # 0x8000_0000


# ----------------------------------------------------------------------------
# UART primitives
# ----------------------------------------------------------------------------
def open_port(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=2.0,
    )
    # discard any stale bytes the FPGA may have dribbled out during reset
    ser.reset_input_buffer()
    return ser


def write_reg(ser: serial.Serial, idx: int, value: int) -> None:
    """Blocking register write: 1 op byte + 4 data bytes MSB-first."""
    assert 0 <= idx < 128
    assert 0 <= value <= 0xFFFFFFFF
    op = 0x00 | (idx & 0x7F)
    payload = bytes([
        op,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >>  8) & 0xFF,
        (value >>  0) & 0xFF,
    ])
    ser.write(payload)
    ser.flush()


def read_reg(ser: serial.Serial, idx: int) -> int:
    """Blocking register read: 1 op byte out, 4 data bytes back MSB-first."""
    assert 0 <= idx < 128
    op = 0x80 | (idx & 0x7F)
    ser.write(bytes([op]))
    ser.flush()
    data = ser.read(4)
    if len(data) != 4:
        raise TimeoutError(f"read_reg({idx}): got {len(data)} bytes, want 4")
    return struct.unpack(">I", data)[0]


def stream_read(ser: serial.Serial, n_bytes: int, per_byte_budget_s: float = 0.001) -> bytes:
    """
    Kick the STREAM read and collect exactly `n_bytes` bytes back.

    per_byte_budget_s is a safety factor for the read timeout:
        at 921600 baud a byte is ~0.011 ms, so 1 ms/byte leaves plenty of
        margin for DDR3 latency, TX FIFO drain, OS jitter, etc.
    """
    op = 0x80 | (STREAM_REG & 0x7F)
    ser.write(bytes([op]))
    ser.flush()

    # give the FPGA enough rope to stream everything
    prev_timeout = ser.timeout
    ser.timeout = max(2.0, n_bytes * per_byte_budget_s + 1.0)
    try:
        data = ser.read(n_bytes)
    finally:
        ser.timeout = prev_timeout

    if len(data) != n_bytes:
        raise TimeoutError(
            f"stream_read: got {len(data)} / {n_bytes} bytes"
        )
    return data


# ----------------------------------------------------------------------------
# ADC instruction encoder (matches adc_core layout)
#   {opcode[3:0], count_field[11:0], delay_or_target[15:0]}
# ----------------------------------------------------------------------------
OP_NOP = 0x0
OP_SAM = 0x1
OP_JMP = 0x2
OP_END = 0xF

ADC_PROGRAM_SLOTS = ADC_CTRL_REG_IDX     # PCs 0..52 are usable; reg[53] = ctrl
ADC_ADDR_WIDTH    = 6                    # $clog2(NUM_REGS=54)
ADC_ADDR_MASK     = (1 << ADC_ADDR_WIDTH) - 1


def make_adc_insn(opcode: int, count_field: int, delay_or_target: int) -> int:
    return (
        ((opcode          & 0xF  ) << 28) |
        ((count_field     & 0xFFF) << 16) |
         (delay_or_target & 0xFFFF)
    )


# ----------------------------------------------------------------------------
# Tiny assembler for ADC programs
#
#   A program is a Python list whose elements are either:
#     * an Insn   (built with nop / sam / jmp / end)
#     * a Label   (built with label("name"))   -- 0-byte placeholder
#
#   assemble(program) returns a list of 32-bit words ready to feed to
#   write_reg() at ADC_CORE_BASE+0, +1, +2, ...  Labels are resolved to PC
#   values (PC is the slot index inside the ADC core, NOT the absolute UART
#   register index -- the JMP target field is core-relative).
#
#   Example -- sample 64 then jump back forever:
#     program = [
#         label("loop"),
#         nop(delay=3),
#         sam(count=64),
#         jmp(target="loop", count=0),    # count=0 -> infinite loop
#         end(),                          # unreachable, but keep it for safety
#     ]
# ----------------------------------------------------------------------------
class Insn:
    __slots__ = ("opcode", "count_field", "delay_or_target", "target_label", "src")
    def __init__(self, opcode, count_field=0, delay_or_target=0, target_label=None, src=""):
        self.opcode          = opcode
        self.count_field     = count_field
        self.delay_or_target = delay_or_target
        self.target_label    = target_label    # str or None
        self.src             = src             # human-readable description
    def __repr__(self):
        return f"Insn({self.src})"


class Label:
    __slots__ = ("name",)
    def __init__(self, name): self.name = name
    def __repr__(self): return f"Label({self.name!r})"


def nop(delay: int = 1) -> Insn:
    assert 0 <= delay <= 0xFFFF, "NOP delay must fit in 16 bits"
    return Insn(OP_NOP, 0, delay, src=f"NOP delay={delay}")

def sam(count: int) -> Insn:
    assert 0 <= count <= 0xFFF, "SAM count must fit in 12 bits"
    return Insn(OP_SAM, count, 0, src=f"SAM count={count}")

def jmp(target, count: int = 0) -> Insn:
    """JMP to `target` (label name OR raw core-PC int).  count=0 means
    'loop forever'; count=N means 'take this branch N times then fall through'."""
    assert 0 <= count <= 0xFFF, "JMP count must fit in 12 bits"
    if isinstance(target, str):
        return Insn(OP_JMP, count, 0, target_label=target,
                    src=f"JMP {target!r} count={count}")
    else:
        assert 0 <= target <= ADC_ADDR_MASK, \
            f"JMP target {target} out of {ADC_ADDR_WIDTH}-bit range"
        return Insn(OP_JMP, count, target & ADC_ADDR_MASK,
                    src=f"JMP pc={target} count={count}")

def end() -> Insn:
    return Insn(OP_END, 0, 0, src="END")

def label(name: str) -> Label:
    return Label(name)


def assemble(program) -> list:
    """Resolve labels and emit a list of 32-bit instruction words."""
    # First pass: assign each Insn a PC; collect label -> PC table
    pcs        = []                    # parallel to insn list
    labels     = {}
    insns      = []
    pc         = 0
    for item in program:
        if isinstance(item, Label):
            if item.name in labels:
                raise ValueError(f"duplicate label {item.name!r}")
            labels[item.name] = pc
        elif isinstance(item, Insn):
            insns.append(item)
            pcs.append(pc)
            pc += 1
        else:
            raise TypeError(f"program element must be Insn or Label, got {type(item)}")

    if pc > ADC_PROGRAM_SLOTS:
        raise ValueError(
            f"program is {pc} insns long but ADC core only has "
            f"{ADC_PROGRAM_SLOTS} slots (reg[{ADC_CORE_BASE}..{ADC_CTRL_ADDR-1}])"
        )

    # Second pass: resolve labels and encode
    words = []
    for ins in insns:
        d = ins.delay_or_target
        if ins.target_label is not None:
            if ins.target_label not in labels:
                raise ValueError(f"undefined label {ins.target_label!r}")
            d = labels[ins.target_label] & ADC_ADDR_MASK
        words.append(make_adc_insn(ins.opcode, ins.count_field, d))
    return words, insns


# ----------------------------------------------------------------------------
# Text-format parser  (.psm files)
#
# Syntax (case-insensitive, comma optional, whitespace flexible):
#
#     ; comments after  ;  #  or  //
#     label_name:                       ; label on its own line
#     label_name: nop 3                 ; label + insn on the same line
#     nop  <delay>                      ; 0..65535 cycles of idle
#     sam  <count>                      ; sample <count> samples (0..4095)
#     jmp  <target> [count]             ; target = label name OR raw PC int
#                                       ; count default = 0  (loop forever)
#     end
#
# Numbers can be:  decimal (12)   hex (0x1f)   binary (0b1010)
#
# Example:
#     ; record 4 blocks of 32 samples = 128 total
#     loop:
#         sam 32
#         jmp loop, 3
#         end
# ----------------------------------------------------------------------------
_NUM_RE   = re.compile(r"^(?:0x[0-9a-fA-F]+|0b[01]+|-?\d+)$")
_LABEL_RE = re.compile(r"^([A-Za-z_]\w*)\s*:\s*(.*)$")


def _strip_comment(line: str) -> str:
    out = line
    for marker in (";", "#", "//"):
        i = out.find(marker)
        if i >= 0:
            out = out[:i]
    return out


def _parse_num(s: str) -> int:
    s = s.strip()
    if s.startswith(("0x", "0X")):
        return int(s, 16)
    if s.startswith(("0b", "0B")):
        return int(s, 2)
    return int(s, 10)


def parse_psm(text: str) -> list:
    """Parse an ADC assembly source string into a program list (Insn|Label)
    that you can hand straight to assemble() or run_program()."""
    items = []
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = _strip_comment(raw).strip()
        if not line:
            continue

        # peel an optional leading "name:" label
        m = _LABEL_RE.match(line)
        if m:
            items.append(label(m.group(1)))
            line = m.group(2).strip()
            if not line:
                continue

        tokens = re.split(r"[\s,]+", line)
        op     = tokens[0].lower()
        args   = tokens[1:]

        try:
            if op == "nop":
                if len(args) != 1:
                    raise ValueError(f"NOP needs 1 arg (delay), got {len(args)}")
                items.append(nop(delay=_parse_num(args[0])))

            elif op == "sam":
                if len(args) != 1:
                    raise ValueError(f"SAM needs 1 arg (count), got {len(args)}")
                items.append(sam(count=_parse_num(args[0])))

            elif op == "jmp":
                if len(args) not in (1, 2):
                    raise ValueError(
                        f"JMP needs 1 or 2 args (target [count]), got {len(args)}"
                    )
                tgt_tok = args[0]
                if _NUM_RE.match(tgt_tok):
                    target = _parse_num(tgt_tok)
                else:
                    target = tgt_tok                       # label name
                cnt = _parse_num(args[1]) if len(args) == 2 else 0
                items.append(jmp(target=target, count=cnt))

            elif op == "end":
                if args:
                    raise ValueError("END takes no args")
                items.append(end())

            else:
                raise ValueError(f"unknown opcode {op!r}")

        except Exception as e:
            raise ValueError(f"line {lineno} {raw.rstrip()!r}: {e}") from None

    return items


def parse_psm_file(path: str) -> list:
    """Read a .psm file from disk and return the parsed program."""
    with open(path, "r", encoding="utf-8") as f:
        return parse_psm(f.read())


# ----------------------------------------------------------------------------
# ADC test flow -- mirror of run_adc_test() in ddr_combine_tb.sv
# ----------------------------------------------------------------------------
def run_adc_test(
    ser: serial.Serial,
    sam_count: int,
    nop_delay: int = 3,
    settle_s: float = 0.2,
    poll_s: float = 0.05,
    poll_tries: int = 60,
) -> bool:
    """
    Run one NOP / SAM(sam_count) / END program, wait for it to finish by
    polling reg[STATUS] until bytes_written stops growing, then stream the
    written region back and sanity-check it against the expected shape.
    """
    print("\n" + "=" * 54)
    print(f"ADC test  SAM count = {sam_count}")
    print("=" * 54)

    exp_txs   = (sam_count + SAMPLES_PER_TX - 1) // SAMPLES_PER_TX
    exp_words = exp_txs * WORDS_PER_TX
    exp_bytes = exp_words * BYTES_PER_WORD

    insn0 = make_adc_insn(OP_NOP, 0,         nop_delay)   # NOP delay
    insn1 = make_adc_insn(OP_SAM, sam_count, 0       )    # SAM count
    insn2 = make_adc_insn(OP_END, 0,         0       )    # END

    # 1. park the core (ctrl = 0) so the following program load is clean
    write_reg(ser, ADC_CTRL_ADDR, 0x0000_0000)

    # 2. load the program
    write_reg(ser, ADC_CORE_BASE + 0, insn0)
    write_reg(ser, ADC_CORE_BASE + 1, insn1)
    write_reg(ser, ADC_CORE_BASE + 2, insn2)

    print(f"  reg[{ADC_CORE_BASE+0}] = 0x{insn0:08x}  (NOP delay={nop_delay})")
    print(f"  reg[{ADC_CORE_BASE+1}] = 0x{insn1:08x}  (SAM count={sam_count})")
    print(f"  reg[{ADC_CORE_BASE+2}] = 0x{insn2:08x}  (END)")
    print(f"  expected txs={exp_txs}, words={exp_words}, bytes={exp_bytes}")

    # 3. fire the ADC
    write_reg(ser, ADC_CTRL_ADDR, ADC_CTRL_START)

    # 4. wait until bytes_written stops growing.  We can't see w_adc_active
    #    on the board, but the status overlay is enough: once the writer has
    #    flushed its last (possibly zero-padded) burst, bytes_written freezes.
    prev = -1
    stable_reads = 0
    for i in range(poll_tries):
        time.sleep(poll_s)
        status = read_reg(ser, STATUS_REG) & 0x1FFF_FFFF    # 29b field
        if status == prev and status > 0:
            stable_reads += 1
            if stable_reads >= 2:                           # 2 consecutive equal reads
                break
        else:
            stable_reads = 0
        prev = status
    else:
        print(f"FAIL: bytes_written never stabilised (last={prev})")
        return False

    hw_bytes_written = status
    print(f"  bytes_written (HW) = {hw_bytes_written}  (expected {exp_bytes})")

    # 5. park the core again and let it settle
    write_reg(ser, ADC_CTRL_ADDR, 0x0000_0000)
    time.sleep(settle_s)

    # 6. sanity check the byte count
    pass_flag = True
    if hw_bytes_written != exp_bytes:
        print(f"FAIL: HW bytes_written={hw_bytes_written} != expected {exp_bytes}")
        pass_flag = False

    # 7. stream the whole region back
    data = stream_read(ser, hw_bytes_written)
    print(f"  streamed back {len(data)} bytes")

    if len(data) % BYTES_PER_WORD != 0:
        print(f"FAIL: stream size {len(data)} not a multiple of {BYTES_PER_WORD}")
        pass_flag = False

    # 8. structural sanity (LAYOUT-AGNOSTIC).
    #    We don't know the exact 8-sample pack order inside a 128b DDR word
    #    (sample[0] could be in bits [127:112] OR [15:0]).  But we DO know:
    #      * Any word that is entirely inside the zero-pad region must be
    #        all-zero bytes, regardless of pack order.
    #      * The last real sample occupies the word at index
    #        floor(sam_count / SAMPLES_PER_WORD).  Everything past that is
    #        guaranteed-zero fill.
    n_words           = len(data) // BYTES_PER_WORD
    first_pad_word    = (sam_count + SAMPLES_PER_WORD - 1) // SAMPLES_PER_WORD
    first_pad_byte    = first_pad_word * BYTES_PER_WORD
    tail              = data[first_pad_byte:]

    if any(b != 0 for b in tail):
        bad_off = next(i for i, b in enumerate(tail) if b != 0)
        print(f"FAIL: zero-pad region not zero "
              f"(first non-zero at stream byte {first_pad_byte + bad_off}, "
              f"expected all-zero from byte {first_pad_byte})")
        pass_flag = False
    else:
        if len(tail) > 0:
            print(f"  zero-pad region OK "
                  f"(bytes {first_pad_byte}..{len(data)-1} all zero, "
                  f"{len(tail)} bytes)")

    # 9. cosmetic: dump the first 8 samples under the Option-A decode assumption
    #    (sample[0] at bits [127:112] of the word, i.e. byte[0:1] of each word).
    #    If these look like garbage, your writer probably uses Option B
    #    (sample[0] at [15:0]) -- swap hi/lo or read within each word in
    #    reverse.  Either way this is NOT a pass/fail criterion.
    head_samples = []
    for s in range(min(8, sam_count)):
        word_idx = s // SAMPLES_PER_WORD
        slot     = s %  SAMPLES_PER_WORD
        off      = word_idx * BYTES_PER_WORD + slot * 2
        hi, lo   = data[off], data[off + 1]
        v        = (hi << 8) | lo
        if v & 0x8000:
            v -= 0x10000
        head_samples.append(v)
    print(f"  first {len(head_samples)} samples (Option-A decode, informational) "
          f"= {head_samples}")

    if pass_flag:
        print(f"PASS  (SAM count={sam_count})")
    else:
        print(f"FAIL  (SAM count={sam_count})")
    return pass_flag


# ----------------------------------------------------------------------------
# run_program -- generic loader for any program built with the assembler
# ----------------------------------------------------------------------------
def _expected_samples(insns) -> int:
    """Static estimate of how many samples a program WILL emit.

    Conservative: NOP/END contribute 0; SAM contributes its count_field;
    JMP with finite count multiplies the preceding loop body; JMP with
    count=0 (infinite) gives up and returns -1, meaning "I can't tell --
    the test will rely solely on the bytes_written-stable poll".

    This is just for sizing the readback / printing the expected value.
    """
    # crude single-loop heuristic -- enough for typical loop forms like
    #   label("L"); ...; sam(N); jmp("L", count=K); end()
    last_sam_block = 0
    body_samples   = 0
    total          = 0
    for ins in insns:
        if ins.opcode == OP_SAM:
            body_samples += ins.count_field
            total        += ins.count_field
        elif ins.opcode == OP_JMP:
            if ins.count_field == 0:
                return -1                                  # infinite
            # JMP fires count_field times before falling through
            total += body_samples * ins.count_field
            body_samples = 0
        elif ins.opcode == OP_NOP:
            pass
        elif ins.opcode == OP_END:
            break
    return total


def run_program(
    ser: serial.Serial,
    program,
    name: str = "program",
    settle_s: float = 0.2,
    poll_s: float = 0.05,
    poll_tries: int = 200,
    max_readback_bytes: int = 2 * 1024 * 1024,
) -> bool:
    """
    Assemble `program`, push it to the FPGA, fire the ADC, wait for
    bytes_written to stabilise, then stream the whole written region back
    and dump basic stats / a hex preview.

    Returns True if no obvious anomaly was hit.  This is a generic runner --
    it does NOT do the SAM-specific zero-pad check that run_adc_test() does
    (a handcoded program might legitimately end on a non-multiple-of-64).
    """
    words, insns = assemble(program)

    print("\n" + "=" * 54)
    print(f"PROGRAM  {name}   ({len(words)} insns)")
    print("=" * 54)
    for pc, (w, ins) in enumerate(zip(words, insns)):
        print(f"  reg[{ADC_CORE_BASE+pc:2d}] = 0x{w:08x}   ; {ins.src}")

    exp_samples = _expected_samples(insns)
    if exp_samples >= 0:
        exp_txs   = (exp_samples + SAMPLES_PER_TX - 1) // SAMPLES_PER_TX
        exp_bytes = exp_txs * BYTES_PER_TX
        print(f"  expected: {exp_samples} samples -> {exp_txs} TXs "
              f"= {exp_bytes} bytes (with zero pad)")
    else:
        exp_bytes = None
        print(f"  expected: program loops forever -- relying on stable poll")

    # 1. park, load, fire
    write_reg(ser, ADC_CTRL_ADDR, 0x0000_0000)
    for pc, w in enumerate(words):
        write_reg(ser, ADC_CORE_BASE + pc, w)
    write_reg(ser, ADC_CTRL_ADDR, ADC_CTRL_START)

    # 2. wait for bytes_written to freeze
    prev = -1
    stable_reads = 0
    for _ in range(poll_tries):
        time.sleep(poll_s)
        status = read_reg(ser, STATUS_REG) & 0x1FFF_FFFF
        if status == prev and status > 0:
            stable_reads += 1
            if stable_reads >= 2:
                break
        else:
            stable_reads = 0
        prev = status
    else:
        print(f"FAIL: bytes_written never stabilised (last={prev}); "
              f"if this program loops, stop it manually and reset.")
        write_reg(ser, ADC_CTRL_ADDR, 0x0000_0000)
        return False

    hw_bytes_written = status
    print(f"  bytes_written (HW) = {hw_bytes_written}")
    if exp_bytes is not None and hw_bytes_written != exp_bytes:
        print(f"  WARN: HW bytes_written != expected {exp_bytes}")

    # 3. park core, settle, stream-read
    write_reg(ser, ADC_CTRL_ADDR, 0x0000_0000)
    time.sleep(settle_s)

    if hw_bytes_written == 0:
        print("  nothing to read back.")
        return True
    if hw_bytes_written > max_readback_bytes:
        print(f"  too big to read back ({hw_bytes_written} bytes > "
              f"{max_readback_bytes}); skipping stream_read")
        return True

    data = stream_read(ser, hw_bytes_written)
    print(f"  streamed back {len(data)} bytes")

    # 4. tiny hex preview of the first/last word
    def _hex(b):
        return " ".join(f"{x:02x}" for x in b)
    head = data[:BYTES_PER_WORD]
    tail = data[-BYTES_PER_WORD:]
    print(f"  first word : {_hex(head)}")
    print(f"  last  word : {_hex(tail)}")

    return True


# ----------------------------------------------------------------------------
# Example programs.  Add your own here, then run with --program <name>.
# ----------------------------------------------------------------------------
def make_example_programs():
    return {
        "single_64": [
            nop(delay=3),
            sam(count=64),
            end(),
        ],
        "single_50": [
            nop(delay=3),
            sam(count=50),
            end(),
        ],
        "loop_4x32": [
            # SAM 32 four times, then end -- 128 samples total.
            # Demonstrates JMP with finite count (loops back 3 more times,
            # then falls through on the 4th visit).
            label("L"),
            sam(count=32),
            jmp("L", count=3),
            end(),
        ],
        "two_blocks": [
            # Two back-to-back SAM blocks of different sizes.
            sam(count=40),
            nop(delay=10),
            sam(count=24),
            end(),
        ],
    }


# ----------------------------------------------------------------------------
# AD9833 quick config (matches the example 4 writes in send_local.py)
# ----------------------------------------------------------------------------
def configure_ad9833(ser: serial.Serial) -> None:
    """Your existing AD9833 boot sequence, rewritten via write_reg()."""
    write_reg(ser, AD9833_BASE + 0, 0x0000_2100)
    write_reg(ser, AD9833_BASE + 1, 0x48D1_4567)
    write_reg(ser, AD9833_BASE + 2, 0x2000_C100)
    write_reg(ser, AD9833_BASE + 3, 0x0000_0001)
    print("AD9833 configured")


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
def main():
    examples = make_example_programs()

    ap = argparse.ArgumentParser()
    ap.add_argument("--port",   default="COM16")
    ap.add_argument("--baud",   type=int, default=921600)
    ap.add_argument("--no-ad9833", action="store_true",
                    help="skip the AD9833 boot sequence")
    ap.add_argument("--sam", type=int, nargs="+", default=None,
                    help="legacy mode: simple NOP/SAM(N)/END runs "
                         "(e.g. --sam 64 50 65 to mirror the TB)")
    ap.add_argument("--program", nargs="+", default=None,
                    metavar="NAME",
                    help=f"run one or more named programs from the assembler; "
                         f"available: {', '.join(examples.keys())}")
    ap.add_argument("--psm", nargs="+", default=None, metavar="PATH",
                    help="run one or more programs read from .psm text files")
    ap.add_argument("--list-programs", action="store_true",
                    help="print the available programs and their assembled "
                         "instruction words, then exit")
    ap.add_argument("--dry-run", action="store_true",
                    help="parse / assemble only, don't open the serial port")
    args = ap.parse_args()

    if args.list_programs:
        for name, prog in examples.items():
            words, insns = assemble(prog)
            print(f"\n{name}:")
            for pc, (w, ins) in enumerate(zip(words, insns)):
                print(f"  reg[{ADC_CORE_BASE+pc:2d}] = 0x{w:08x}   ; {ins.src}")
        sys.exit(0)

    if args.sam is None and args.program is None and args.psm is None:
        # default behaviour preserved: run the original 3-SAM regression
        args.sam = [64, 50, 65]

    # ---- dry-run: parse + assemble only, dump words, don't touch UART ------
    if args.dry_run:
        if args.program:
            for name in args.program:
                if name not in examples:
                    print(f"unknown program {name!r}")
                    continue
                words, insns = assemble(examples[name])
                print(f"\n{name}:")
                for pc, (w, ins) in enumerate(zip(words, insns)):
                    print(f"  reg[{ADC_CORE_BASE+pc:2d}] = 0x{w:08x}   ; {ins.src}")
        if args.psm:
            for path in args.psm:
                prog = parse_psm_file(path)
                words, insns = assemble(prog)
                tag = os.path.basename(path)
                print(f"\n{tag}:")
                for pc, (w, ins) in enumerate(zip(words, insns)):
                    print(f"  reg[{ADC_CORE_BASE+pc:2d}] = 0x{w:08x}   ; {ins.src}")
        sys.exit(0)

    ser = open_port(args.port, args.baud)
    print(f"Opened {args.port} @ {args.baud} baud")

    try:
        if not args.no_ad9833:
            configure_ad9833(ser)

        all_pass = True

        if args.sam:
            for n in args.sam:
                ok = run_adc_test(ser, sam_count=n)
                all_pass = all_pass and ok

        if args.program:
            for name in args.program:
                if name not in examples:
                    print(f"unknown program {name!r}; "
                          f"available: {', '.join(examples.keys())}")
                    all_pass = False
                    continue
                ok = run_program(ser, examples[name], name=name)
                all_pass = all_pass and ok

        if args.psm:
            for path in args.psm:
                try:
                    prog = parse_psm_file(path)
                except (OSError, ValueError) as e:
                    print(f"FAIL: could not parse {path}: {e}")
                    all_pass = False
                    continue
                ok = run_program(ser, prog, name=os.path.basename(path))
                all_pass = all_pass and ok

        print("\n" + "=" * 54)
        print("ALL TESTS PASS" if all_pass else "SOME TESTS FAILED")
        print("=" * 54)
        sys.exit(0 if all_pass else 1)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
