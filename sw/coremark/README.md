# CoreMark

[upstream/](upstream/) contains the benchmark sources and
[license and usage terms](upstream/LICENSE.md). [umbra/](umbra/) contains the
platform port, startup code and linker script.

Run from the repository root:

```sh
make coremark MODE=performance ITERATIONS=1
make coremark MODE=validation ITERATIONS=1
```

The runner checks the expected CRCs for both seed modes. These short simulation
runs are functional checks, not qualifying CoreMark scores or hardware measurements.

See [running checks](../../docs/IMPLEMENTATION.md#running-checks) for the required tools.
