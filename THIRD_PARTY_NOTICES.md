# Third-party notices

UMBRA's original code uses the [Apache License 2.0](https://github.com/HeNing45/UMBRA/blob/main/LICENSE).
Third-party files keep their own licenses and copyright notices.

- **CoreMark:** from [EEMBC](https://github.com/eembc/coremark).
  See the included [license and usage terms](sw/coremark/upstream/LICENSE.md).
- **Embench-IoT:** from [Embench](https://github.com/embench/embench-iot).
  The suite is distributed under GPLv3, with individual source headers
  specifying the terms for each component. See [COPYING](sw/embench/upstream/COPYING).
  The Apache licence for UMBRA's original code does not replace those terms.
  Embench is a separate software workload, not part of the CPU RTL.
  The MIT-marked `md5sum/md5.c` and `tarfind/tarfind.c` retain their upstream
  author credits; the [MIT licence text](sw/embench/LICENSES/MIT.txt) is also included.
- **Berkeley Spike:** from [riscv-isa-sim](https://github.com/riscv-software-src/riscv-isa-sim).
  Installed separately, not bundled here.

The GDS uses OSU gscl45nm layouts from FreePDK45 1.4, licensed under Apache 2.0.
Copyright 2008 James Stine, Ivan Castellanos and Oklahoma State University.
`physical/umbra_syn_island.gds` includes modified layouts: layer remapping,
intracell metal changes and added well fill.
