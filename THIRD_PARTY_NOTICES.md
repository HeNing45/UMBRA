<!--
SPDX-FileCopyrightText: Copyright 2026 He Ning
SPDX-License-Identifier: Apache-2.0
-->

# Third-party notices

UMBRA is a mixed-license repository. Original RTL, testbenches, verification
tools and documentation are Copyright 2026 He Ning, licensed under
[Apache-2.0](LICENSE). See [NOTICE](NOTICE). Third-party material retains the
terms and attribution listed below; the root licence does not replace them.

## CoreMark®

[CoreMark®](https://github.com/eembc/coremark) is from the Embedded
Microprocessor Benchmark Consortium (EEMBC). Its software is Apache-2.0;
use of the mark is also subject to the COREMARK® Acceptable Use Agreement.
Both texts are in [LICENSE.md](sw/coremark/upstream/LICENSE.md). CoreMark is
a trademark of EEMBC.

The [UMBRA platform port](sw/coremark/umbra/) is original Apache-2.0 code.
The published measurements are CRC checks and iterations per million timed
RTL cycles, not official CoreMark scores or hardware benchmark results.
Keep the EEMBC notices, [run/reporting rules](sw/coremark/upstream/README.md)
and [source-revision record](sw/coremark/upstream/ORIGIN.md) with these files.

## Embench-IoT

[Embench-IoT](https://github.com/embench/embench-iot) includes work by
Embecosm Limited, the University of Bristol and other credited contributors.
Its suite and the [UMBRA port](sw/embench/umbra/) are GPL-3.0-or-later;
see [COPYING](sw/embench/upstream/COPYING) and the individual source headers.
The port is compiled or textually included with the GPL support sources.
The resulting workload images are GPL combined works. Redistributing these
sources or binaries requires compliance with the applicable GPLv3 terms,
including the corresponding-source requirements for binary distribution.
Embench is a software workload, not CPU RTL; this does not relicense the
independent UMBRA CPU under GPL.

Keep these additional per-file notices:

- **MD5:** [md5sum/md5.c](sw/embench/upstream/src/md5sum/md5.c) is based on
  Creationix's [MD5 implementation](https://gist.github.com/creationix/4710780),
  modified by Julian Kunkel for Embench. It is marked MIT.
- **TAR search:** [tarfind/tarfind.c](sw/embench/upstream/src/tarfind/tarfind.c),
  created by Julian Kunkel for Embench, is marked MIT. The
  [MIT licence text](sw/embench/LICENSES/MIT.txt) accompanies both files;
  retain their source credits as well.
- **Depthwise convolution:** [depthconv/depthconv.c](sw/embench/upstream/src/depthconv/depthconv.c)
  is Copyright 2024 The TensorFlow Authors, Apache-2.0. Embench extracted
  the kernel and unit test and converted them from C++ to C. Its header
  records those changes. The [Apache-2.0 terms](LICENSE) apply to that component.
- **picojpeg:** the original Rich Geldreich implementation carries a
  public-domain notice inside the source. The Embench version also carries
  GPL-3.0-or-later notices. Preserve both; do not describe the whole Embench
  wrapper as public domain. See [libpicojpeg.c](sw/embench/upstream/src/picojpeg/libpicojpeg.c).
- **sglib:** [sglib.h](sw/embench/upstream/src/sglib-combined/sglib.h) retains
  Marian Vittek's original terms and Embench's GPL-3.0-or-later notice.
  The original terms permit verbatim use, including commercial use, while
  derivatives require an open-source/GPL licence or the author's permission.
  This is not permission to incorporate a modified sglib into a proprietary
  binary. Preserve the complete original header and the Embench notices.

## Berkeley Spike

[Spike](https://github.com/riscv-software-src/riscv-isa-sim) is installed
separately. Its source and executable are not bundled in this repository.

## OSU cells and FreePDK45

The released GDS includes OSU gscl45nm cell geometry from FreePDK45 1.4,
licensed under Apache-2.0. Copyright 2008 James Stine, Ivan Castellanos
and Oklahoma State University. Retain this attribution and the Apache licence
when redistributing the layout.

`physical/umbra_syn_island.gds` is a release-only artifact, not a tracked git
file. Download it from the [physical release](https://github.com/HeNing45/UMBRA/releases/tag/physical-osu45-20ns-frontend).
It contains modified layouts: layer remapping, intracell metal changes and
added well fill. Modified intracell metal was not recharacterized.

Provenance: FreePDK45 1.4 / OSU gscl45nm; exact cell tarball hash not recorded
in this repository. A complete pinned transformation recipe is not included
here, so the published tree alone is insufficient to reproduce those changes.
This is an academic PDK, not a foundry kit; the GDS is not fabricated silicon.

Future GDS releases must include the current `LICENSE`, `NOTICE` and
`THIRD_PARTY_NOTICES.md`. Existing releases retain the notices attached when
they were published; this update does not replace their assets.
