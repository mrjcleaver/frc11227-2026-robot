# Command-Based Programming

This robot uses WPILib's **command-based paradigm** — the standard for FRC Java robots. Understanding it is essential for reading or writing any robot code.

---

## The Core Idea

Robot code is split into two kinds of objects that have one job each:

- **Subsystems** own hardware and expose actions
- **Commands** use subsystems to perform those actions

Nothing else runs logic. `RobotContainer` wires them together. The `CommandScheduler` runs everything.

---

## Subsystems

A subsystem is a class that owns a set of hardware (motors, sensors, actuators) and nothing outside that class should touch that hardware directly.

Subsystems expose their capabilities as factory methods that return `Command` objects:

```java
// IntakeSubsystem owns the roller motor and angle motor.
// It doesn't run them directly — it describes how to run them.
public Command intakeDown(double speed) {
    return runOnce(() -> angleMotor.set(speed));
}

public Command intakeBalls() {
    return run(() -> rollerMotor.set(IntakeConstants.intakingRollerSpeed));
}
```

The subsystem also declares a **default command** — what it does when nothing else is using it. For the drivetrain this is field-centric joystick drive.

This robot has three subsystems:

| Subsystem | Hardware owned |
|---|---|
| `CommandSwerveDrivetrain` | 4 swerve modules (8 TalonFX), odometry, vision fusion |
| `ShooterSubsystem` | 6 TalonFX (flywheels + feeders) |
| `IntakeSubsystem` | 1 SparkMax (roller), 1 TalonFX (angle) |

---

## Commands

A command is a unit of robot behavior. It has four lifecycle methods:

```
initialize()  — runs once when the command starts
execute()     — runs every 20 ms while the command is active
isFinished()  — returns true when the command should stop
end(interrupted) — runs once when the command stops (normally or cancelled)
```

Commands declare which subsystems they **require**. The scheduler enforces that only one command can use a subsystem at a time — if a new command requires a subsystem that's already in use, the running command is interrupted.

Most of the time you don't write command classes directly. WPILib provides factory methods on `Subsystem` and `Commands` that build them inline:

```java
// runs execute() once then ends
runOnce(() -> motor.set(speed))

// runs execute() every loop until interrupted
run(() -> motor.set(speed))

// runs execute() every loop until condition is true
runEnd(() -> motor.set(speed), () -> motor.set(0))
    .until(() -> sensor.get())
```

---

## Composition

The real power is combining small commands into larger behaviors without writing new classes:

```java
Commands.sequence(a, b, c)       // run a, then b, then c
Commands.parallel(a, b, c)       // run a, b, c simultaneously
Commands.deadline(main, bg)      // run bg until main finishes, then stop both
Commands.race(a, b)              // run both, stop when either finishes
a.withTimeout(3.0)               // cancel a after 3 seconds
a.andThen(b)                     // same as sequence(a, b), inline
a.until(() -> condition)         // cancel a when condition becomes true
a.onlyIf(() -> condition)        // skip a entirely if condition is false at start
```

Example — the 2-piece auto is readable as plain English:

```java
Commands.sequence(
    shooter.shootSequence().withTimeout(3.0),        // shoot preloaded piece
    intake.intakeDown(speed).withTimeout(1.0),        // deploy intake
    Commands.deadline(                               // drive to piece...
        drivetrain.followPath("Left Path"),          //   ...path is the main command
        intake.intakeBalls()                         //   ...rollers run alongside
    ),                                               //   stops when path ends
    intake.intakeUp(speed).withTimeout(0.75),        // retract intake
    drivetrain.aimAtTarget().withTimeout(1.5),       // aim at target
    shooter.shootSequence().withTimeout(3.0)         // shoot second piece
)
```

---

## The Scheduler

`CommandScheduler.getInstance()` is a singleton that runs every 20 ms robot loop tick. It:

1. Runs `execute()` on every active command
2. Checks `isFinished()` — ends commands that return true
3. Enforces subsystem requirements — interrupts conflicting commands
4. Runs the default command for any idle subsystem

You never call `execute()` yourself. You schedule a command and the scheduler takes it from there.

---

## Button Bindings

In `RobotContainer.configureBindings()`, controller buttons are bound to commands using trigger methods:

```java
joystick.y().whileTrue(command)   // command runs while button is held, cancels on release
joystick.a().onTrue(command)      // command starts on button press, runs to completion
joystick.back().toggleOnTrue(cmd) // alternates between starting and cancelling
```

Binding a button does not run the command — it registers a trigger. The scheduler checks triggers every loop and schedules or cancels commands accordingly.

---

## RobotContainer

`RobotContainer` is the assembly point. It:

1. Instantiates all subsystems
2. Binds buttons to commands via `configureBindings()`
3. Provides `getAutonomousCommand()` to `Robot.java`

It does not run any logic itself. It is called once at robot startup.

---

## Where PathPlanner Fits

PathPlanner integrates cleanly because `AutoBuilder.followPath()` and `AutoBuilder.buildAuto()` return standard WPILib `Command` objects. They compose with shooter and intake commands exactly like anything else:

```java
Commands.sequence(
    shooter.shootSequence(),          // WPILib command
    AutoBuilder.followPath(path),     // PathPlanner command — same interface
    shooter.shootSequence()
)
```

Named commands (`NamedCommands.registerCommand(...)`) are the bridge in the other direction — they let PathPlanner's `.auto` files trigger WPILib commands by name at waypoints along a path.

---

## Mental Model

```
Hardware
  └── Subsystem  (owns hardware, exposes Command factories)
        └── Command  (uses subsystem, describes behavior)
              └── Composition  (sequence, parallel, deadline...)
                    └── CommandScheduler  (runs everything, 20 ms loop)
                          └── RobotContainer  (wires it all together)
```

The discipline that makes this work: **subsystems own hardware, commands own behavior, nothing else does either.**
