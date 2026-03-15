# Auto Planning Checklist

Work through these in order. Later steps depend on earlier ones being solid.

---

## 1. Wire `Autos.java` into `RobotContainer`

`Autos.java` exists but is not yet connected. The existing `autoChooser` in `RobotContainer` is a plain PathPlanner chooser with no named commands registered. Until this step is done, the programmatic auto routines and named command markers will not be available.

In `RobotContainer.java`, make the following changes:

**Add a field:**
```java
private final Autos autos;
```

**Replace the existing auto chooser lines in the constructor:**
```java
// Remove these two lines:
autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Auto Mode", autoChooser);

// Replace with:
autos = new Autos(drivetrain, shooter, intake);
```

**Remove the now-unused field declaration:**
```java
// Remove this line from the field declarations:
private final SendableChooser<Command> autoChooser;
```

**Update `getAutonomousCommand()`** to call `autos.getSelected()` instead of `autoChooser.getSelected()`:
```java
return Commands.sequence(visionReset, autos.getSelected());
```

> **Important:** `Autos.java` registers named commands via `NamedCommands.registerCommand()` before calling `AutoBuilder.buildAutoChooser()`. This ordering is required — if you restructure the constructor, named commands must still be registered first or PathPlanner will not find them at runtime.

- [ ] `RobotContainer` updated as above
- [ ] `./gradlew build` passes
- [ ] Only one auto chooser appears on SmartDashboard (under "Auto Chooser")
- [ ] All expected options are listed: Do Nothing, Just Shoot, Shoot and Taxi (Left/Right), 2-Piece (Left/Right), 3-Piece

---

## 2. Verify Code Compiles and Deploys

- [ ] `./gradlew build` passes with no errors
- [ ] Code deploys to robot without errors
- [ ] `Autos.java` named commands register without crashing (check Driver Station log)
- [ ] "Auto Chooser" appears on SmartDashboard with all expected options

---

## 2. Hardware Checks (Robot on Blocks)

### Limelight
- [ ] Limelight is powered and accessible at `limelight.local`
- [ ] AprilTag detections appear in NetworkTables (`tx`, `ty`, `ta`, `tid`)
- [ ] `getBotPoseEstimate_wpiBlue` returns a valid pose when a tag is in view
- [ ] Tag ID 10 (hub tag) is visible from each planned starting position on the field

### Drivetrain
- [ ] All four swerve modules respond correctly (no reversed motors, no wrong encoder offsets)
- [ ] Field-centric heading resets correctly with left bumper
- [ ] Odometry pose shown on SmartDashboard "Field" widget moves in the right direction

### Shooter
- [ ] Both flywheels spin up and reach target RPS at each distance in the lerp table
- [ ] Feeder runs at 30 RPS
- [ ] `shootSequence()` completes without timeout in under 3 s

### Intake
- [ ] Intake deploys and retracts fully within the 1 s / 0.75 s timeouts
- [ ] Hard stop detection triggers correctly (stator current ≥ 95 A) — mechanism stops, does not stall
- [ ] `intakeBalls()` picks up a game piece reliably

---

## 3. Measure and Tune Timeouts

These values in `Autos.java` are estimates. Measure on the real robot and update them.

| Command | Current timeout | How to measure |
|---|---|---|
| `shootSequence()` | 3.0 s | Time from command start to piece leaving robot |
| `intakeDown()` | 1.0 s | Time for full deploy to hard stop |
| `intakeUp()` | 0.75 s | Time for full retract to hard stop |
| `aimAtTarget()` | 1.5 s | Time for TX to settle within ±1° |

Add ~0.2 s margin to each measured time.

---

## 4. Create PathPlanner Paths

Open PathPlanner and create the following path files. Starting poses must match actual robot placement — measure from field drawings.

### Required paths

| File | Starting zone | Purpose |
|---|---|---|
| `Left Path.path` | Left side of alliance station | Taxi + 2-piece left |
| `Right Path.path` | Right side of alliance station | Taxi + 2-piece right |
| `Full Path.auto` | Center | 3-piece |

### For each path
- [ ] Starting pose set to actual robot placement (measure, don't guess)
- [ ] Path avoids field obstacles and other robots in expected positions
- [ ] Velocity and acceleration constraints set — start conservatively (e.g., 2 m/s, 2 m/s²) and increase after testing
- [ ] Alliance mirroring verified: run path on Red alliance and confirm it mirrors correctly

### For 2-piece paths: add event markers
Place these markers at the correct positions along the path:

| Marker name | Position |
|---|---|
| `IntakeDown` | ~0.5 s before reaching the game piece |
| `Intake` | At the game piece pickup zone |
| `IntakeUp` | After pickup, before returning to shoot position |

### For Full Path.auto: add markers for both pickups
Repeat the above marker sequence for each game piece along the route, then add `AimAndShoot` at each shoot position.

---

## 5. Tune PathPlanner PID

Current values: translation (10, 0, 0), rotation (7, 0, 0).

Test with a simple straight-line path first.

| Symptom | Fix |
|---|---|
| Robot overshoots endpoint | Reduce translation kP (try 7) |
| Robot oscillates at endpoint | Add translation kD (try 0.1) |
| Robot heading oscillates | Reduce rotation kP (try 5) |
| Robot drifts sideways on straight path | Check odometry — likely encoder/module issue |
| Robot follows path but is consistently offset | Check starting pose alignment |

---

## 6. Tune Limelight Aim

`aimAtTarget()` uses a P controller with `kP = 0.04` (in `CommandSwerveDrivetrain`).

- [ ] Confirm Limelight sees the hub tag from the shoot position for each auto route
- [ ] Check `targetingAngularVelocity` on NetworkTables while running `aimAtTarget()` — should approach 0 as robot aligns
- [ ] If robot oscillates: reduce `kP` toward 0.02
- [ ] If robot is slow to aim: increase `kP` toward 0.06, or reduce `aimAtTarget()` timeout
- [ ] Acceptable aim error: TX within ±1.5° before shooting

---

## 7. Vision Pose Reset Verification

The auto start pose reset only fires if the Limelight sees a tag. Verify this works before relying on it.

- [ ] Place robot at each starting position and confirm `tagCount > 0` in NetworkTables
- [ ] Confirm pose on the "Field" widget jumps to the correct position when auto is enabled
- [ ] If no tag is visible from a starting position, that auto route must rely on driver placement accuracy — document this

---

## 8. Test Each Routine in Order

Test on the actual field if possible; carpet variation affects path following.

- [ ] **Just Shoot** — robot shoots without moving. Verify piece exits robot.
- [ ] **Shoot and Taxi (Left)** — robot shoots, follows Left Path. Verify it stays in bounds.
- [ ] **Shoot and Taxi (Right)** — same for Right Path.
- [ ] **2-Piece (Left)** — robot shoots, drives to piece, intakes, aims, shoots. Verify both shots score.
- [ ] **2-Piece (Right)** — same for Right Path.
- [ ] **3-Piece (Full Path)** — full routine. Only attempt after 2-piece is consistent.

For each routine, check:
- Robot ends in a safe position (not blocking alliance partners)
- Timing leaves margin — routine should finish before auto period ends
- No commands time out unexpectedly (check Driver Station log)

---

## 9. Competition Preparation

- [ ] Agree on primary and backup auto routines with drive team
- [ ] Label starting positions on robot bumpers to match path starting poses
- [ ] Verify SmartDashboard auto chooser is visible from driver station laptop
- [ ] Test alliance color flipping: run a path declared for Blue while connected as Red alliance
- [ ] Run full routine 3× consecutively — consistency matters more than peak performance

---

## Known Limitations and Risks

| Risk | Mitigation |
|---|---|
| No tag visible at start | Driver must place robot precisely; use tape marks on floor |
| Intake misses game piece | Slow down approach segment of path; widen intake if possible |
| Flywheel not at speed before shoot | Increase `shootSequence()` timeout; check PID tuning |
| Path wrong for Red alliance | Test mirroring explicitly — do not assume it works |
| Java GC pause delays first path | PathPlanner warmup command is already scheduled in `RobotContainer` |
