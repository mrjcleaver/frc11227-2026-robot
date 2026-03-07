// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int intakeLiftMotor = 14;
    public static final int leftFlywheelLead = 16;
    public static final int leftFlywheelFollow = 17;
    public static final int leftFLywheelFeeder = 18;

    public static final int rightFlywheelLead = 19;
    public static final int rightFlywheelFollow = 20;
    public static final int rightFlywheelFeeder = 21;
  }

  public static class ShooterConstants {
    public static final double flywheel_kS = 0.1;
    public static final double flywheel_kV = 0.12;
    public static final double flywheel_kP = 0.11;
    public static final double flywheel_kI = 0;
    public static final double flywheel_kD = 0;

    public static final double feeder_kS = 0;
    public static final double feeder_kV = 0;
    public static final double feeder_kP = 0.1;
    public static final double feeder_kI = 0;
    public static final double feeder_kD = 0;
  }
}
