# README Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the root README with a practical ROS 2 Jazzy guide for installing, running, and checking the ARS430 radar driver.

**Architecture:** Keep documentation in the repository root so GitHub visitors see the current ROS 2 workflow first. The guide documents the Phase 1 single-radar pipeline, launch arguments, verification commands, hardware checks, and troubleshooting without changing runtime code.

**Tech Stack:** Markdown, ROS 2 Jazzy, colcon, libpcap, radar_driver launch and nodes.

---

## File Structure

- Modify: `README.md` as the primary user guide.
- Do not modify runtime code.
- Do not commit generated `build/`, `install/`, or `log/` directories.

---

### Task 1: Replace Root README With ROS 2 Guide

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Write the README**

Replace the existing two-line README with sections for overview, architecture, requirements, installation, build verification, running the driver, manual node commands, radar connection checklist, troubleshooting, and known limitations.

- [ ] **Step 2: Review README content**

Run: read `README.md` and verify it mentions ROS 2 Jazzy, `libpcap0.8-dev`, `colcon build --packages-select radar_driver`, `single_radar.launch.py`, `radar_publisher`, `radar_processor`, `radar_visualizer`, `ip link`, `tcpdump`, and `ros2 topic list`.

- [ ] **Step 3: Verify project still builds and tests pass**

Run:

```bash
colcon build --packages-select radar_driver
python3 -m pytest radar_driver/tests/test_critical_regressions.py -v
```

Expected: build finishes and static tests pass.

- [ ] **Step 4: Commit and push**

Run:

```bash
git add README.md docs/superpowers/plans/2026-06-03-readme-guide.md
git commit -m "Document ROS2 radar driver usage"
git push
```

Expected: commit is pushed to `driver_radar/main`.

---

## Self-Review

- Spec coverage: The plan covers the requested README guide, verification, commit, and push.
- Placeholder scan: No placeholders remain.
- Type consistency: No code types are introduced.
