#!/usr/bin/env bash
set -e
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cd "$(dirname "$0")/.."
cmake --build build >/dev/null
# Netlist: MC6850 /IRQ (open-drain, active-low) -> pull-up -> inverter -> RST7.5
cat > build/irq.net <<'EOF'
pull IRQ_L up
wire IRQ_L mc6850@0xDE.IRQ
gate inv U1 IRQ_L RST_IN
wire RST_IN cpu.RST7.5
EOF
# Program: RST7.5 vector is 0x003C. Main enables MC6850 RX int + EI, then spins.
# The ISR writes 0xAA to the UART (txlog) as a sentinel, then HLT.
# Main @0x0000:
#   3E 15 D3 DE   ctrl=0x15 (RX int enabled, /RTS low)   -- CR bit7=1 enables RX int? use 0x95
#   FB            EI
#   00 C3 06 00   NOP; JMP 0x0006 (spin)   [loop @0x0006]
# NOTE: set CR so RX interrupt is enabled (bit7=1): use 0x95 not 0x15.
# ISR @0x003C: 3E AA D3 DF 76  (MVI A,0xAA; OUT 0xDF; HLT)
# (python3 is not available in this environment, so the image -- equivalent to
#  a bytearray(0x40) with the same three slice assignments -- is built with
#  raw printf byte-writes instead: 9 program bytes, 51 zero-padding bytes up
#  to the RST7.5 vector at 0x003C, then the 5-byte ISR.)
{
  printf '\x3E\x95\xD3\xDE\xFB\x00\xC3\x06\x00'
  head -c 51 /dev/zero
  printf '\x3E\xAA\xD3\xDF\x76'
} > build/irq.bin
rm -f build/irq.txt build/irq_neg.txt
# Feed one RX byte (0x41) so the ACIA raises /IRQ.
./build/i8085-trace.exe -q -S -n 20000 -l 0x0 -e 0x0 \
  --netlist build/irq.net \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;rx=41;txlog=build/irq.txt" build/irq.bin >/dev/null 2>&1
got=$(xxd -p build/irq.txt)
echo "ISR sentinel = $got (expect aa)"
[ "$got" = "aa" ] || { echo FAIL; exit 1; }

# NEGATIVE check: same netlist + program, but NO RX byte fed. With the /IRQ
# line correctly modeled as open-drain (idle => hi-Z => pull-up holds IRQ_L
# high => inverter drives RST_IN low => no RST7.5), the ISR must never run,
# so the sentinel must be ABSENT from the txlog.
./build/i8085-trace.exe -q -S -n 20000 -l 0x0 -e 0x0 \
  --netlist build/irq.net \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;txlog=build/irq_neg.txt" build/irq.bin >/dev/null 2>&1
gotneg=$(xxd -p build/irq_neg.txt)
echo "ISR sentinel (no RX byte) = ${gotneg:-<empty>} (expect NOT aa)"
[ "$gotneg" != "aa" ] || { echo FAIL; exit 1; }

echo PASS
