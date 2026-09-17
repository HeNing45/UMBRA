<!--
SPDX-FileCopyrightText: Copyright 2026 He Ning
SPDX-License-Identifier: Apache-2.0
-->

# CoreMark®-derived CRC and RTL-cycle checks

[upstream/](upstream/) contains the benchmark sources and
[license and usage terms](upstream/LICENSE.md). [umbra/](umbra/) contains the
platform port, startup code and linker script. CoreMark is a trademark of EEMBC.

The upstream software is Apache-2.0; use of the mark is also subject to the
COREMARK® Acceptable Use Agreement. The original UMBRA platform port is
Copyright 2026 He Ning, Apache-2.0. See the [run/reporting rules](upstream/README.md)
and [source-revision record](upstream/ORIGIN.md). These terms are separate from
the GPL-3.0-or-later Embench workload elsewhere in this mixed-license repository.

To reproduce the reported `max` CRC/cycle sample, run from the repository root:

```sh
make coremark MEMORY=delayed PROFILE=max MODE=performance ITERATIONS=16
make coremark MEMORY=delayed PROFILE=max MODE=validation ITERATIONS=1
```

The runner checks the expected CRCs for both seed modes. These short simulation
runs are functional checks, **not an official CoreMark score** or on-board
measurements. The sample is shorter than the required ten seconds. Ratios are
reported as iterations per million timed RTL cycles, not the official score
format. The port deliberately returns zero from `time_in_secs` to prevent
the upstream framework from reporting a qualifying time-based result.

Use `PROFILE=o2` for the comparison profile. Omitting the options selects
`MEMORY=zero PROFILE=o2`, which does not reproduce the reported sample.

See [running checks](../../docs/IMPLEMENTATION.md#running-checks) for the required tools.
