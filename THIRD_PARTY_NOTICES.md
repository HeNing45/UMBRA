# Third party notices

UMBRA's original code uses the [Apache License 2.0](https://github.com/HeNing45/UMBRA/blob/main/LICENSE).
Third party files keep their own licenses and copyright notices.

- **CoreMark:** from [EEMBC](https://github.com/eembc/coremark).
  See the included [license and usage terms](sw/coremark/upstream/LICENSE.md).
- **Embench-IoT:** from [Embench](https://github.com/embench/embench-iot).
  See [COPYING](sw/embench/upstream/COPYING) and the individual source headers.
- **Berkeley Spike:** from [riscv-isa-sim](https://github.com/riscv-software-src/riscv-isa-sim).
  Installed separately, not bundled here.

The GDS uses OSU gscl45nm layouts from FreePDK45 1.4, licensed under Apache 2.0.
Copyright 2008 James Stine, Ivan Castellanos and Oklahoma State University.
`physical/umbra_syn_island.gds` includes modified layouts: layer remapping,
intracell metal changes and added well fill.
