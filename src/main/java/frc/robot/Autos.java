package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Autos {

    private final SendableChooser<Command> chooser;

    public Autos(CommandSwerveDrivetrain drivetrain,
                 ShooterSubsystem shooter,
                 IntakeSubsystem intake) {

        // Register named commands so PathPlanner .auto files can invoke them.
        // Must be registered before AutoBuilder.buildAutoChooser() is called.
        NamedCommands.registerCommand("Shoot",
            shooter.shootSequence().withTimeout(3.0));

        NamedCommands.registerCommand("IntakeDown",
            intake.intakeDown(IntakeConstants.intakeRotateSpeed));

        NamedCommands.registerCommand("IntakeUp",
            intake.intakeUp(IntakeConstants.intakeRotateSpeed));

        NamedCommands.registerCommand("Intake",
            intake.intakeBalls().withTimeout(3.0));

        NamedCommands.registerCommand("AimAndShoot",
            Commands.sequence(
                drivetrain.aimAtTarget().withTimeout(1.5),
                shooter.shootSequence().withTimeout(3.0)
            ));

        chooser = AutoBuilder.buildAutoChooser("Do Nothing");

        chooser.addOption("Just Shoot",
            justShoot(shooter));

        chooser.addOption("Shoot and Taxi (Left)",
            shootAndTaxi(shooter, "Left Path"));

        chooser.addOption("Shoot and Taxi (Right)",
            shootAndTaxi(shooter, "Right Path"));

        chooser.addOption("2-Piece (Left)",
            twoPiece(shooter, intake, drivetrain, "Left Path"));

        chooser.addOption("2-Piece (Right)",
            twoPiece(shooter, intake, drivetrain, "Right Path"));

        chooser.addOption("3-Piece (Full Path)",
            threePiece(shooter));

        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public Command getSelected() {
        Command selected = chooser.getSelected();
        if (selected == null) {
            DriverStation.reportWarning("Auto chooser returned null — running Do Nothing", false);
            return Commands.none();
        }
        return selected;
    }

    // ── Path loading helpers ──────────────────────────────────────────────────

    /**
     * Loads a PathPlanner path file by name, falling back to justShoot if the
     * file is missing. Missing path files throw at construction time otherwise,
     * which crashes the entire robot program before auto starts.
     */
    private Command loadPath(String pathName, ShooterSubsystem shooter) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load path '" + pathName + "': " + e.getMessage(), false);
            return justShoot(shooter);
        }
    }

    /**
     * Loads a PathPlanner .auto file by name, falling back to justShoot if the
     * file is missing.
     */
    private Command loadAuto(String autoName, ShooterSubsystem shooter) {
        try {
            return AutoBuilder.buildAuto(autoName);
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load auto '" + autoName + "': " + e.getMessage(), false);
            return justShoot(shooter);
        }
    }

    // ── Individual Auto Routines ──────────────────────────────────────────────

    /** Shoot the preloaded piece and stay put. */
    private Command justShoot(ShooterSubsystem shooter) {
        return shooter.shootSequence().withTimeout(3.0);
    }

    /** Shoot preloaded piece, then follow a path to leave the starting zone. */
    private Command shootAndTaxi(ShooterSubsystem shooter, String pathName) {
        return Commands.sequence(
            shooter.shootSequence().withTimeout(3.0),
            loadPath(pathName, shooter)
        );
    }

    /**
     * Shoot preloaded piece, drive to pick up a second piece while intaking,
     * then aim and shoot again.
     */
    private Command twoPiece(ShooterSubsystem shooter,
                              IntakeSubsystem intake,
                              CommandSwerveDrivetrain drivetrain,
                              String pathName) {
        return Commands.sequence(
            shooter.shootSequence().withTimeout(3.0),
            intake.intakeDown(IntakeConstants.intakeRotateSpeed).withTimeout(1.0),
            Commands.deadline(
                loadPath(pathName, shooter),
                intake.intakeBalls()
            ),
            intake.intakeUp(IntakeConstants.intakeRotateSpeed).withTimeout(0.75),
            drivetrain.aimAtTarget().withTimeout(1.5),
            shooter.shootSequence().withTimeout(3.0)
        );
    }

    /**
     * Full 3-piece auto using the pre-planned "Full Path" .auto file.
     * Named commands embedded in the path handle intake/shoot triggers.
     */
    private Command threePiece(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.shootSequence().withTimeout(3.0),
            loadAuto("Full Path", shooter)
        );
    }
}
