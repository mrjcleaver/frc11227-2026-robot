package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANBus kCanivoreBus = new CANBus("theGoose");

    final TalonFX m_leftFlywheelLead = new TalonFX(CAN.leftFlywheelLead, kCanivoreBus);
    final TalonFX m_leftFlywheelFollow = new TalonFX(CAN.leftFlywheelFollow, kCanivoreBus);
    final TalonFX m_leftFlywheelFeeder = new TalonFX(CAN.leftFLywheelFeeder, kCanivoreBus);

    final TalonFX m_rightFlywheelLead = new TalonFX(CAN.rightFlywheelLead, kCanivoreBus);
    final TalonFX m_rightFlywheelFollow = new TalonFX(CAN.rightFlywheelFollow, kCanivoreBus);
    final TalonFX m_rightFlywheelFeeder = new TalonFX(CAN.rightFlywheelFeeder, kCanivoreBus);

    final VelocityVoltage m_VelocityVoltageRequest = new VelocityVoltage(0).withSlot(0);

    final Slot0Configs flywheelSlot0Configs = new Slot0Configs();
    final Slot0Configs feederSlot0Configs = new Slot0Configs();

    public static double limelightDeg = 5.0; 
    public static double limelightHeightIn = 28.0; 
    public static double goalHeightIn = 44.25;

    public ShooterSubsystem() {
        // Check constants.java file to see the values provided
        flywheelSlot0Configs.kS = ShooterConstants.flywheel_kS; // Add 0.1 V output to overcome static friction
        flywheelSlot0Configs.kV = ShooterConstants.flywheel_kV; // A velocity target of 1 rps results in 0.12 V output
        flywheelSlot0Configs.kP = ShooterConstants.flywheel_kP; // An error of 1 rps results in 0.11 V output
        flywheelSlot0Configs.kI = ShooterConstants.flywheel_kI; // no output for integrated error
        flywheelSlot0Configs.kD = ShooterConstants.flywheel_kD; // no output for error derivative

        m_leftFlywheelLead.getConfigurator().apply(flywheelSlot0Configs);
        m_leftFlywheelFollow.getConfigurator().apply(flywheelSlot0Configs);
        m_rightFlywheelLead.getConfigurator().apply(flywheelSlot0Configs);
        m_rightFlywheelFollow.getConfigurator().apply(flywheelSlot0Configs);

        m_rightFlywheelLead.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        m_rightFlywheelFeeder.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        // set follow flywheels to follow their leader motors
        m_rightFlywheelFollow.setControl(new Follower(m_rightFlywheelLead.getDeviceID(), MotorAlignmentValue.Aligned));
        m_leftFlywheelFollow.setControl(new Follower(m_leftFlywheelLead.getDeviceID(), MotorAlignmentValue.Aligned));

        feederSlot0Configs.kS = ShooterConstants.feeder_kS;
        feederSlot0Configs.kV = ShooterConstants.feeder_kV;
        feederSlot0Configs.kP = ShooterConstants.feeder_kP;
        feederSlot0Configs.kI = ShooterConstants.feeder_kI;
        feederSlot0Configs.kD = ShooterConstants.feeder_kD;

        m_leftFlywheelFeeder.getConfigurator().apply(feederSlot0Configs);
        m_rightFlywheelFeeder.getConfigurator().apply(feederSlot0Configs);
    }

    public static double getDistance() {
        double ty = LimelightHelpers.getTY("limelight");
        double limelightSightHeight = goalHeightIn - limelightHeightIn;
        double dist = limelightSightHeight / Math.tan(limelightDeg + ty);
        distancePub.set(dist);
        return dist;
    }

    public Command spinFlywheel(DoubleSupplier speed) {
        return this.runEnd(
            () -> {
                // set velocity to 8 rps, add 0.5 V to overcome gravity
                m_rightFlywheelLead.setControl(m_VelocityVoltageRequest.withVelocity(8).withFeedForward(0.5));
                m_leftFlywheelLead.setControl(m_VelocityVoltageRequest.withVelocity(8).withFeedForward(0.5));

                // set velocity to 2 rps, add 0.5 V to overcome gravity
                m_leftFlywheelFeeder.setControl(m_VelocityVoltageRequest.withVelocity(2).withFeedForward(0.5));
                m_rightFlywheelFeeder.setControl(m_VelocityVoltageRequest.withVelocity(2).withFeedForward(0.5));
            },
            () -> {
                m_rightFlywheelLead.stopMotor();
                m_leftFlywheelLead.stopMotor();
                m_rightFlywheelFeeder.stopMotor();
                m_leftFlywheelFeeder.stopMotor();
            });
    }
}
