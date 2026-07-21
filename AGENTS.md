## Repository Guidance

Build and dependencies
- Prefer the Docker + Makefile workflow documented in `README.md`.
- Firmware builds normally run via `make firmware`.
- Host benchmark work normally runs via `make benchmark`.
- Local CMake builds require `PICO_SDK_PATH` to be set.
- Third-party dependencies under `include/` are git submodules. Keep them initialized with `git submodule update --init --recursive`.

Repository infrastructure
- Treat dependency wiring, Docker setup, and debug/flash flows as shared firmware infrastructure. Do not change them unless the task requires it.
- Keep third-party dependency management consistent with the repository’s submodule-based model under `include/`.

PR review workflows
- Firmware-specific `codex-review` guidance lives in `docs/codex-review-policy.md`.
- Shared review and PR workflow policy lives in `tekkura/.github`:
  https://github.com/tekkura/.github/blob/main/docs/pr-workflow.md
