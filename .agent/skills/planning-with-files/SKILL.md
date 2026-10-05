---
name: planning-with-files
description: Planning workflow using structured files for tasks, findings, and progress.
---

# Planning with files

Use the following files when the task is non-trivial:
- task_plan.md: scope, steps, acceptance criteria
- findings.md: root cause, evidence, decisions
- progress.md: current status and next action

## Rules
- Keep plan files short and explicit.
- Update them when the plan changes.
- Separate facts from assumptions.
- Close tasks only after verification.

## Typical flow
1. Create or update task_plan.md.
2. Record findings.md as evidence emerges.
3. Update progress.md after each meaningful verification step.
4. Keep final state aligned with the project goals.

## Project-specific reminder
This project is safety-critical embedded software. Planning must reflect hardware, serial protocol, and timing constraints.
