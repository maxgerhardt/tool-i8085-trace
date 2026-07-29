# tool-i8085-trace

The **i8085-trace** Intel 8085 instruction-level simulator, packaged for
[PlatformIO](https://platformio.org/) and used by
[`platform-intel_mcs85`](https://github.com/maxgerhardt/platform-intel_mcs85) as
a simulator (`upload_protocol = i8085-trace`) target and, optionally, as a GDB
debug server.

## Contents

```
bin/
  i8085-trace.exe          the simulator (statically linked -- no runtime DLLs)
  i8085-console.exe        raw-mode stdin<->TCP bridge, auto-spawned per windowed UART
plugins/
  mc6850.dll               MC6850 ACIA (UART), relocatable via base=; matches the
                           OMEN ALPHA console UART at 0xDE/0xDF. Pure I/O.
  memory.dll               memory-region plugin: ROM / write-protected EEPROM,
                           enforced through the on_mem_write hook
```

## Simulator usage

```sh
# run a flat binary loaded/entered at a given address
bin/i8085-trace.exe -l 0x2000 -e 0x2000 program.bin

# with the console ACIA plugin, capturing UART TX to a file
bin/i8085-trace.exe -q -S -n 4000000 -l 0x2000 -e 0x2000 \
    --io-plugin=plugins/mc6850.dll \
    --io-plugin-config="base=0xDE;txlog=uart.txt" program.bin

# GDB remote debugging (RSP) server on a port
bin/i8085-trace.exe --gdb=1234 -l 0x2000 -e 0x2000 program.bin
```

The MC6850 plugin models an ACIA at ports `base` (status / control) and
`base+1` (data) — `base=0xDE` matches the OMEN ALPHA console UART — keeping TDRE
asserted so firmware TX proceeds, and appends every transmitted byte to the
`txlog` file. Multiple `--io-plugin=…/mc6850.dll` instances at different `base`
ports model several UARTs. This is how a firmware's UART output is surfaced when
running under the simulator.

See the simulator's own `--help` for the full option set (interrupts, memory
dumps, coverage, tracepoints, ...).

## Provenance

The core 8085 emulation is derived from
[sim8085](https://github.com/debjitbis08/sim8085) by Debjit Biswas (BSD
3-Clause). Windows build (MinGW-w64, UCRT).

## License

BSD-3-Clause (see the upstream i8085-trace project).
