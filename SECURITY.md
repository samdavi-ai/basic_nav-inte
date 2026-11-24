# Security policy

## Scope

This repository contains desktop simulation, ROS 2 integration, adapter authorization, and robot safety code. A Gazebo test does not establish the safety of a physical robot deployment.

## Reporting a vulnerability

Please report suspected vulnerabilities through GitHub's private vulnerability reporting for this repository when available. Include the affected path, conditions required to reproduce the issue, and whether the report involves credentials, authorization, data retention, or the motion/safety path. Do not open a public issue containing credentials, personal data, or an unmitigated path to robot motion.

## Repository handling

- Do not commit credentials, tokens, personal biometric data, or private adapter grant files.
- Keep motion requests behind the registered adapter and kernel authorization flow.
- Do not disable E-stop, collision monitoring, command arbitration, or obstacle sensors to make a test pass.
- Treat all physical actuation and deployment as a separate, explicitly authorized engineering activity.
