---
name: karpathy-guidelines
description: Coding discipline for small, reliable, goal-driven changes.
---

# Karpathy coding guidelines

## Principles
- Think before coding: understand the bug or feature before editing.
- Keep changes small and surgical.
- Prefer the simplest correct solution.
- Validate with the smallest relevant check.
- Avoid broad refactors unless necessary.

## Workflow
1. State the actual problem in one sentence.
2. Read only the files needed to confirm root cause.
3. Patch the smallest possible scope.
4. Run a focused verification.
5. Stop when the issue is fixed and proven.

## Project-specific rules
- Preserve embedded safety constraints.
- Prefer stack/static allocation over heap in ISRs.
- Avoid blocking calls in loops; prefer non-blocking timing.
- Do not change GPIO mapping without explicit approval.
- Keep serial protocol compatibility intact.
