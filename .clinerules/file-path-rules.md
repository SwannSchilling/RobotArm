# File Path Rules

This document defines the file path conventions and best practices for the RobotArm project.

---

## 1. Path Format

- Use **forward slashes** `/` and **relative paths**
- Always prefer relative paths over absolute paths
- Example: Use `can_driver.py` instead of `/home/pi/RobotArm/can_driver.py`

## 2. Tool Path Parameter

- Always provide paths **relative** to the current working directory `/home/pi/RobotArm`
- When using tools like `read_file`, `write_to_file`, `replace_in_file`, etc., use relative paths from the root of the project

## 3. Workspace Syntax

- Use `@workspace:path` format when referencing files across workspaces
- Example: `@RobotArm:src/index.ts` refers to `src/index.ts` in the RobotArm workspace

## 4. Naming Conventions

Follow consistent naming patterns:

| Pattern | Description | Example |
| --- | --- | --- |
| `can_*.py` | CAN-related modules | `can_driver.py`, `can_diagnostics.py` |
| `*Test*.py` | Test files | `AX12A_Test.py`, `can_test_01.py` |
| `*Controller*.py` | Controller modules | `WaveshareServoController.py` |
| `*Adapter*.py` | Adapter/wrapper modules | `WaveshareServoAdapter.py` |

## 5. Directory Structure

Organize files within the project structure:

- **Root Level** — Main Python files (e.g., `can_driver.py`, `app.py`, `servo_can.py`)
- **Subdirectories** — Grouped modules (e.g., `AmazingHand/MoveHand.py`, `AmazingHand/HandController.ino`)

## 6. Avoid Hardcoded Absolute Paths

In file operations (`read_file`, `write_to_file`, `replace_in_file`, etc.):

- ❌ Bad: `<path>/home/pi/RobotArm/can_driver.py</path>`
- ✅ Good: `<path>can_driver.py</path>`

## 7. No `cd` or `~` Usage

- Stick to relative paths from the working directory `/home/pi/RobotArm`
- Avoid using `cd` to change directories in commands
- Avoid using `~` to reference the home directory

## 8. Context Awareness

When creating new files or referencing existing ones:

- Consider the project's file structure
- Place files in the appropriate directory (root or subdirectory)
- Reference files using their relative path from the working directory

---

### Quick Reference

| Scenario | Correct Format |
| --- | --- |
| Root file | `can_driver.py` |
| Subdirectory file | `AmazingHand/MoveHand.py` |
| Workspace reference | `@RobotArm:can_driver.py` |
| Avoid | `/home/pi/RobotArm/can_driver.py` |
| Avoid | `~/RobotArm/can_driver.py` |