# Codex PR Review Policy

This file contains firmware-specific PR review guidance. Shared PR workflow and Codex review behavior lives in `tekkura/.github`:
https://github.com/tekkura/.github/blob/main/docs/pr-workflow.md

## Correctness
- Ensure build, benchmark, and protocol/documentation changes remain consistent with the implementation.
- Be explicit about unsupported commands, partial implementations, benchmark gaps, and protocol mismatches.
- When raising a finding, prefer to also suggest a concrete fix, preferred resolution, or next verification step so the review is actionable instead of purely critical.

## Review style
- Findings should be specific and technically grounded.
- Prefer actionable review comments over purely descriptive criticism.
- When possible, point to the likely remediation path, not just the symptom.
