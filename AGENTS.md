## Repository Guidance

Build and dependencies
- Prefer the Docker + Makefile workflow documented in `README.md`.
- Firmware builds normally run via `make firmware`.
- Host benchmark work normally runs via `make benchmark`.
- Local CMake builds require `PICO_SDK_PATH` to be set.
- Third-party dependencies under `include/` are git submodules. Keep them initialized with `git submodule update --init --recursive`.

Change scope and safety
- Keep changes focused on the task at hand. Do not widen scope with unrelated refactors or workflow churn.
- Treat dependency wiring, Docker setup, and debug/flash flows as shared repo infrastructure. Do not change them unless the task requires it.
- Keep third-party dependency management consistent with the repository’s submodule-based model under `include/`.

Firmware debugging expectations
- For MAX77958, USB-PD, startup tests, UART logging, queue handling, and interrupt-driven behavior, analyze the state machine and shared wait/interrupt paths before proposing changes.
- Treat a failing test, log line, or event as a reproducer for the underlying firmware issue until proven otherwise.
- Do not hide, gate, skip, or weaken startup tests unless explicitly instructed.
- Prefer bounded waits with diagnostics and shared recovery paths over blocking polls or per-test workarounds.
- Keep debug logging useful, but do not block normal init or `on_start` behavior.

PR review workflows
- `codex-review` and `codex-reply` policy lives in `docs/codex-review-policy.md`.
- Milestone-specific review guidance lives under `docs/milestones/`.
