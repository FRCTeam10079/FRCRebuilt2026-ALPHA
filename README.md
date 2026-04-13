# FRC Team 10079 - 2026 Alpha Robot Code

[![Java 17](https://img.shields.io/badge/Java-17-007396?logo=openjdk&logoColor=white)](https://adoptium.net/)
[![WPILib 2026](https://img.shields.io/badge/WPILib-2026-blue)](https://github.com/wpilibsuite/allwpilib)
[![Formatting](https://github.com/FRCTeam10079/FRCRebuilt2026-ALPHA/actions/workflows/format.yaml/badge.svg)](https://github.com/FRCTeam10079/FRCRebuilt2026-ALPHA/actions/workflows/format.yaml)
![Language](https://img.shields.io/badge/Language-Java%2099.9%25-orange)
![Status](https://img.shields.io/badge/Status-Archived-lightgrey)

Early-season alpha robot codebase for FRC Team 10079 (Arrowdynamics), 2026 season. This repo covers the initial hardware bring-up and subsystem development phase before work moved to the competition codebase ([FRCRebuilt2026-COMP](https://github.com/FRCTeam10079/FRCRebuilt2026-COMP)).

> [!NOTE]
> This is the **alpha/development bot** repository. It was active from mid-January to mid-February 2026 and is now archived. It is **not** the competition code. See the COMP repo linked above for that.

---

## Overview

This codebase was the team's first working version of the 2026 robot software stack. It served as the testbed where subsystems were brought up on physical hardware for the first time, initial vision integration was validated, and the core command structure was established. Once the foundation was solid enough, development continued in the dedicated competition repo.

Key differences from the COMP repo:
- Simpler flat subsystem structure (no separated `statemachine/`, `constants/`, `auto/`, `util/` packages)
- No Choreo trajectory library or SOTM/SOTF shot compensation
- No AdvantageKit IO layer abstraction. Direct hardware calls
- `RobotStateMachine.java` present at the top level but early-stage
- `NetworkedLib` for runtime-adjustable values (replaced by AdvantageKit tunable numbers in COMP)

---

## Package Structure

```
src/main/java/frc/robot/
├── commands/              # Driver-triggered and helper commands
├── generated/             # CTRE Tuner X generated swerve constants
├── lib/
│   └── NetworkedLib/      # Runtime value updating over NetworkTables
├── pathfinding/           # Early pathfinding framework
├── subsystems/
│   ├── CommandSwerveDrivetrain.java
│   ├── IndexerSubsystem.java
│   ├── IntakeSubsystem.java
│   ├── LimelightSubsystem.java
│   ├── ShooterSubsystem.java
│   └── SwerveHeadingController.java
├── Constants.java
├── LimelightHelpers.java
├── Robot.java
├── RobotContainer.java
├── RobotStateMachine.java
└── Telemetry.java
```

---

## Subsystems

|         Subsystem         |                                             Description                                              |
|---------------------------|------------------------------------------------------------------------------------------------------|
| `CommandSwerveDrivetrain` | CTRE Phoenix6 swerve drive with field-centric teleop and heading controller integration.             |
| `ShooterSubsystem`        | Flywheel shooter bring-up: motor control, RPM targeting, early pivot work.                           |
| `IndexerSubsystem`        | Fuel feed path from intake to shooter. Tested on hardware and confirmed working (PR #18).            |
| `IntakeSubsystem`         | Intake roller control. Motor control added and tested on robot (PR #17).                             |
| `LimelightSubsystem`      | Initial Limelight vision integration and pose observation ingestion. Fixed and stabilized in PR #28. |
| `SwerveHeadingController` | PID-based heading lock for the drivetrain, used during aim and auto align workflows.                 |

---

## Quick Start

### Prerequisites

- JDK 17
- WPILib 2026 toolchain

### Build / Check / Test / Simulate

Windows (PowerShell/CMD):

```bash
.\gradlew.bat build
.\gradlew.bat spotlessCheck
.\gradlew.bat test
.\gradlew.bat simulateJava
```

macOS/Linux:

```bash
./gradlew build
./gradlew spotlessCheck
./gradlew test
./gradlew simulateJava
```

### Deploy to RoboRIO

```bash
.\gradlew.bat deploy
```

---

## Development Timeline

|     Date     |                                                Milestone                                                 |
|--------------|----------------------------------------------------------------------------------------------------------|
| Mid-Jan 2026 | Initial project setup, formatter config (Seth), CTRE swerve scaffolding                                  |
| Jan 25–26    | AprilTag map added, pose tracking restored, vision fixes                                                 |
| Jan 27       | Indexer tested on robot and working (PR #18); intake motor control added (PR #17)                        |
| Jan 30–31    | Migration to WPILib 2026                                                                                 |
| Feb 1        | Intake bind added (PR #27)                                                                               |
| Feb 3–4      | Vision fixed (PR #28); NetworkedLib for runtime value updates added (PR #24); autodrive feature (PR #31) |
| Feb 6–11     | Shooter subsystem bring-up iterations (PRs #36, #37 revert, #41)                                         |
| ~Feb 11      | Development shifted to competition repo                                                                  |

---

## Contributors

| Contributor |                            GitHub                            |                                     Involvement                                     |
|-------------|--------------------------------------------------------------|-------------------------------------------------------------------------------------|
| Risith      | [@risithcha](https://github.com/risithcha)                   | Core architect: swerve, vision, state machine, repo structure, and primary reviewer |
| Team        | [@ArrowDynamics10079](https://github.com/ArrowDynamics10079) | Hardware bring-up: shooter, indexer, intake testing on physical robot               |
| Aaryan      | [@Acer-15](https://github.com/Acer-15)                       | Intake motor control (PR #17), intake binds (PR #27)                                |
| Nethul      | [@Thatcoder321](https://github.com/Thatcoder321)             | Indexer bring-up and testing (PR #18)                                               |
| Seth        | [@SethMortenson64](https://github.com/SethMortenson64)       | Formatter setup and CI tooling (PRs #9, #25)                                        |
| Fish        | [@sockeye-d](https://github.com/sockeye-d)                   | Removed pre-commit hooks, added formatter CI workflow (PR #25)                      |
| Dhruv       | [@Duve3](https://github.com/Duve3)                           | NetworkedLib for runtime-adjustable values (PR #24)                                 |

---

## License

Distributed under the WPILib BSD-style license. See `WPILib-License.md`.
