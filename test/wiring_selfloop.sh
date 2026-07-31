#!/usr/bin/env bash
set -e
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cd "$(dirname "$0")/.."
cmake --build build >/dev/null
# Netlist: tie an i8255 Port A OUTPUT pin (A0) through a buffer to Port C INPUT
# pin C0, so writing PA bit0=1 should appear on PC bit0 read back by the CPU.
cat > build/self.net <<'EOF'
wire NA i8255@0x00.A0
gate buf B1 NA NC
wire NC i8255@0x00.C0
EOF
# Program: mode-set A output, C-lower input (0x81); PA=0x01; read PC -> UART.
#  3E 81 D3 03  3E 01 D3 00  DB 02 D3 DF  76
# NOTE: NO i8255 ctrl= pre-config. The 8255 powers on all-input; the guest's own
# mode-set (OUT 0x03,0x81) flips Port A to output at RUNTIME, and the net tracks
# that direction change live -- so A0 becomes a driver with no load-time hint.
printf '\x3e\x81\xd3\x03\x3e\x01\xd3\x00\xdb\x02\xd3\xdf\x76' > build/self.bin
rm -f build/self.txt
./build/i8085-trace.exe -q -S -n 200 -l 0x0 -e 0x0 \
  --netlist build/self.net \
  --io-plugin=./build/i8255.dll  --io-plugin-config="base=0x0" \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;txlog=build/self.txt" build/self.bin >/dev/null 2>&1
got=$(xxd -p build/self.txt)
echo "PC readback = $got (expect 01)"
[ "$got" = "01" ] || { echo FAIL; exit 1; }
echo PASS
