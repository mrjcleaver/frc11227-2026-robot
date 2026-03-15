# Autonomous

## How It Works

Autonomous is built on [PathPlanner](https://pathplanner.dev), which generates trajectories and drives the robot along them using a holonomic PID controller. PathPlanner path files (`.path`) and auto files (`.auto`) live in `src/main/deploy/pathplanner/`.

At the start of every auto, the robot reads its position from the Limelight and resets odometry if an AprilTag is visible. This corrects for placement error before any movement begins.

```
Auto start
  └─ Vision pose reset (if tag visible)
       └─ Run selected auto routine
```

## Pose Reset

Implemented in `RobotContainer.getAutonomousCommand()`. Before handing off to the PathPlanner routine, a one-shot command reads `getBotPoseEstimate_wpiBlue("limelight")` and calls `drivetrain.resetPose()` if `tagCount > 0`. If no tag is visible the reset is skipped and odometry is left as-is.

This is the single most impactful correction for path-following accuracy. A robot placed 5 cm off from its expected starting pose will carry that error through the entire auto without it.

## Mid-Path Vision Correction

During path following, the Limelight's MegaTag2 pose estimates are continuously fused into the drivetrain's Kalman filter via `addVisionMeasurement()` (called from `Robot.periodic()`). This provides soft, ongoing correction as long as AprilTags are in view — it does not hard-reset pose, so it cannot cause sudden jumps.

## PathPlanner Configuration

| Parameter | Value |
|---|---|
| Translation PID | 10, 0, 0 |
| Rotation PID | 7, 0, 0 |
| Alliance flipping | Automatic (Red mirrors Blue paths) |
| Pose supplier | `drivetrain.getState().Pose` |
| Speed supplier | `drivetrain.getState().Speeds` |

## Named Commands

These are registered in `Autos.java` and can be used as event markers inside any PathPlanner `.auto` file.

| Name | What it does |
|---|---|
| `Shoot` | Runs `shootSequence()`, timeout 3 s |
| `IntakeDown` | Deploys intake to down position |
| `IntakeUp` | Retracts intake to up position |
| `Intake` | Runs full `intakeBalls()` sequence, timeout 3 s |
| `AimAndShoot` | Aims via Limelight (1.5 s), then shoots (3 s) |

## Auto Routines

Routines are available in the `Autos` chooser on SmartDashboard under **"Auto Chooser"**.

### Do Nothing
Does nothing. Default selection to prevent accidents.

### Just Shoot
Shoots the preloaded game piece and stays put.

```
shootSequence (3 s max)
```

Use when: mobility points are not achievable or the robot needs to stay in position.

### Shoot and Taxi (Left / Right)
Shoots the preloaded piece, then drives along the named path to leave the starting zone.

```
shootSequence (3 s max)
  └─ followPath("Left Path" or "Right Path")
```

Use when: mobility points matter but a multi-piece auto is not reliable.

### 2-Piece (Left / Right)
Shoots preloaded piece, drives to collect a second piece while the intake runs, then aims and shoots.

```
shootSequence (3 s max)
  └─ intakeDown (1 s max)
       └─ followPath + intakeBalls in parallel
            └─ intakeUp (0.75 s max)
                 └─ aimAtTarget (1.5 s max)
                      └─ shootSequence (3 s max)
```

The path drives to the game piece; `intakeBalls()` runs alongside it via `Commands.deadline()` and stops when the path ends. The robot then retracts, aims with the Limelight, and fires.

Use when: the robot reliably picks up the second piece and two-piece auto is worth the risk.

### 3-Piece (Full Path)
Shoots the preloaded piece, then runs the `Full Path.auto` file from PathPlanner. Intake and shoot triggers mid-path are handled by the named commands embedded in the `.auto` file.

```
shootSequence (3 s max)
  └─ AutoBuilder.buildAuto("Full Path")
       [IntakeDown marker] → intake deploys
       [Intake marker]     → rollers run
       [IntakeUp marker]   → intake retracts
       [AimAndShoot marker] → aim + shoot
       ... repeated for third piece
```

Use when: practice time has proven the full routine is consistent.

## Aiming During Auto

`drivetrain.aimAtTarget()` holds the robot stationary and rotates toward the Limelight target using a proportional controller (`kP = 0.04`). It returns a `Command` and should always be bounded with `.withTimeout()`.

The controller output is scaled by `MaxAngularRate` (0.75 rot/s) before being sent to the swerve modules.

## Tuning Notes

- **Shooter timeouts** (currently 3 s): measure actual spinup time from rest and set to spinup + 0.5 s margin.
- **Intake deploy timeout** (currently 1 s): should be just long enough for the mechanism to reach hard stop.
- **Aim timeout** (currently 1.5 s): if the Limelight consistently loses the target, reduce and accept a slightly less accurate shot.
- **Translation/Rotation PID**: if the robot overshoots path endpoints, reduce translation kP from 10. If it oscillates during rotation, reduce rotation kP from 7.

## Path Files

| File | Used by |
|---|---|
| `Left Path.path` | Shoot and Taxi (Left), 2-Piece (Left) |
| `Right Path.path` | Shoot and Taxi (Right), 2-Piece (Right) |
| `Full Path.auto` | 3-Piece |

Path files are edited in the PathPlanner desktop application. Start poses must match actual robot placement on the field. Alliance color flipping is handled automatically at runtime.
