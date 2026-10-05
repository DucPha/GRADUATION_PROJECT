---
name: superpowers
description: Structured debugging and verification protocol for code fixes.
---

# Superpowers debugging protocol

## Before fixing
- Reproduce the bug or confirm the failing condition.
- Read the exact error or stack trace.
- Trace data flow to the root cause.
- Check the last meaningful change.

## While fixing
- Make one hypothesis at a time.
- Patch the root cause, not the symptom.
- Keep the change localized.
- Preserve interfaces and serial/device contracts.

## After fixing
- Run the smallest relevant test or compile check.
- Confirm the real behavior changed for the better.
- Watch for regressions in timing, memory, and safety-critical logic.

## Embedded caution
- Sensor timeout and NaN handling must be safe.
- Motor stall and obstacle avoidance must fail gracefully.
- Verify resource usage on ESP32 memory limits.
