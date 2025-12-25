---
id: 1
title: Tasks To Issues Unavailable
stage: misc
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude User
command: /sp.taskstoissues
labels: [infrastructure, unavailable, github]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - history/prompts/general/1-taskstoissues-unavailable.misc.prompt.md
tests:
 - none
---

## Prompt

```
$ARGUMENTS
```

## Response snapshot

The /sp.taskstoissues command requires specific infrastructure (PowerShell scripts and GitHub MCP server) that is not available in this environment. The command is designed to parse tasks.md files and create corresponding GitHub issues via the MCP server, but this functionality is unavailable without the proper setup.

A tasks.md file was found at ./specs/001-physical-ai-ros2/tasks.md, but the infrastructure needed to convert tasks to GitHub issues is not accessible in this execution environment.

## Outcome

- ✅ Impact: Identified infrastructure requirements for task-to-issue conversion
- 🧪 Tests: none
- 📁 Files: PHR documenting unavailable functionality
- 🔁 Next prompts: Use environment with full Spec-Kit Plus infrastructure
- 🧠 Reflection: Command requires MCP server and PowerShell scripts

## Evaluation notes (flywheel)

- Failure modes observed: Missing infrastructure for GitHub MCP integration
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Execute in environment with MCP server