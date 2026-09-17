# CoreMark® upstream files

The C bodies and `coremark.h` body are retained from EEMBC CoreMark commit
`cfa9ab377835911f23d9b0831c7be302ed1f58de`. UMBRA's platform port is in
[`../umbra/`](../umbra/).

On 2026-09-17, the initial licence headers in those six files were replaced
with the matching official Apache-2.0 headers, including EEMBC's copyright
and author credit. No algorithm or framework code was changed.

The headers, complete [`LICENSE.md`](LICENSE.md) and [`README.md`](README.md)
come from [EEMBC CoreMark commit 1f483d5b8316753a742cbf5590caf5bd0a4e4777](https://github.com/eembc/coremark/tree/1f483d5b8316753a742cbf5590caf5bd0a4e4777).
The official branch at retrieval was `main`. This does not make the retained
C bodies a snapshot of that newer revision. Relative links in the upstream
README refer to the full EEMBC repository; its platform directories and build
system are not bundled here.

The upstream README includes the run and reporting rules. UMBRA publishes
CRC checks and timed RTL-cycle measurements, not official CoreMark® scores.
CoreMark is a trademark of EEMBC.
