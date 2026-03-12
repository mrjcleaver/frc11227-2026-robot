// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class CAN {
    // CANivore bus
    public static final int intakeAngle = 14;
    public static final int leftFlywheelLead = 16;
    public static final int leftFlywheelFollow = 17;
    public static final int leftFLywheelFeeder = 18;

    public static final int rightFlywheelLead = 19;
    public static final int rightFlywheelFollow = 20;
    public static final int rightFlywheelFeeder = 21;

    // RIO bus
    public static final int pdh = 1;
    public static final int intakeRollers = 2;
  }

  public static class IntakeConstants {
    // TODO: Intake angle PID values should be stored here along with gear ratio
    public static final double intakeRotateCurrentLimit = 95;
    public static final double intakeRotateSpeed = 0.3;
    public static final int intakeUpDirection = -1;
    public static final int intakeDownDirection = 1;

    public static final double intakingRollerSpeed = 0.7;
    public static final double intakingPosition = -0.01;

    public static final double jiggleFrequency = 1;
    public static final double jiggleAmplitude = 0.06;
    public static final double jiggleOffset = 0.07;
    public static final double jiggleRollerSpeed = 0.5;
  }

  public static class ShooterConstants {
    public static final double flywheel_kS = 3.72;
    public static final double flywheel_kV = 0.015;
    public static final double flywheel_kP = 8;
    public static final double flywheel_kI = 0;
    public static final double flywheel_kD = 0;

    public static final double feeder_kS = 24;
    public static final double feeder_kV = 0.2;
    public static final double feeder_kP = 8;
    public static final double feeder_kI = 0;
    public static final double feeder_kD = 0;

    public static final double feederSetpointRPS = 30;

    public static final InterpolatingDoubleTreeMap lerpTable = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.57, 47.0),
      Map.entry(2.76, 55.0),
      Map.entry(3.0, 57.0),
      Map.entry(3.13, 60.0),
      Map.entry(3.8, 64.5),
      Map.entry(5.0, 73.0)
    );
  }

  public static class RobotConstants {
    public static final double limelightHeightInches = 28.0;
    public static final double limelightDegrees = 10.0;    
  }

  public static class FieldConstants {
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static double hubTagHeight = fieldLayout.getTags().get(10).pose.getZ();
  }
}
