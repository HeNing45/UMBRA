# CoreMark

[upstream/](upstream/) contains the benchmark sources and
[license and usage terms](upstream/LICENSE.md). [umbra/](umbra/) contains the
platform port, startup code and linker script.

To reproduce the reported `max` sample, run from the repository root:

```sh
make coremark MEMORY=delayed PROFILE=max MODE=performance ITERATIONS=16
make coremark MEMORY=delayed PROFILE=max MODE=validation ITERATIONS=1
```

The runner checks the expected CRCs for both seed modes. These short simulation
runs are functional checks, not qualifying CoreMark scores or on-board measurements.

Use `PROFILE=o2` for the comparison profile. Omitting the options selects
`MEMORY=zero PROFILE=o2`, which does not reproduce the reported sample.

See [running checks](../../docs/IMPLEMENTATION.md#running-checks) for the required tools.
