# Security Policy

## Reporting a vulnerability

pybox3d's realistic security surface is memory safety in the hand-written
C extension (`libbox3d/`, `src/pybox3d/_ext/`) -- there's no network I/O,
authentication, or file-format parsing of untrusted input anywhere in the
library. If you've found a crash, out-of-bounds access, use-after-free, or
similar memory-safety issue (especially one reachable from pure-Python
input, e.g. a constructor argument that segfaults the interpreter rather
than raising a Python exception), please report it privately rather than
opening a public issue:

- Use GitHub's private vulnerability reporting: go to the
  [Security tab](https://github.com/algaves/pybox3d/security) of this
  repository and select **"Report a vulnerability."**

Please include:
- A minimal reproducing Python snippet (or C repro if it's in `libbox3d`
  directly).
- The pybox3d version, Python version, OS/platform, and whether you
  installed a prebuilt wheel or built from source.
- Whatever crash output/traceback you have (a segfault won't give you a
  Python traceback -- a core dump or `gdb`/`lldb` backtrace is still
  useful if you have one).

## Supported versions

This project is pre-1.0 (see [TODO.md](TODO.md)/[docs/roadmap.md](docs/roadmap.md)
for what's landed) with no semantic-versioning commitment yet and a single
rolling release line -- there's no supported-version matrix to consult.
Security fixes land against the latest release; if you're on an older
version, please upgrade before reporting to confirm the issue still
reproduces.

## Scope

Out of scope: issues that require the caller to already run arbitrary
Python code in the same process (that caller already has full control),
and denial-of-service via pathological but *valid* physics scenes (e.g. an
intentionally huge body count causing slow steps) -- see
[docs/limitations.md](docs/limitations.md) for this library's documented
performance characteristics (naive-adjacent sort-and-sweep broad phase,
not a production-grade broad phase for very large scenes).
