package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

    final VelocityTorqueCurrentFOC m_velocityTorqueRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
    final VelocityVoltage m_VelocityVoltageRequest = new VelocityVoltage(0).withSlot(0);

    final Slot0Configs flywheelSlot0Configs = new Slot0Configs();
    final Slot0Configs feederSlot0Configs = new Slot0Configs();

    final DutyCycleOut m_DutyCycle = new DutyCycleOut(0.0);

    final DoublePublisher flywheelSpeedPub;
    final DoublePublisher distancePub;
    final DoublePublisher limelightTYPub;
    final DoublePublisher heightDiffPub;

    public static double limelightDeg = 5.0; 
    public static double limelightHeightIn = 28.0; 
    public static double goalHeightIn = 44.25;

    public ShooterSubsystem(DoubleTopic flywheelVelocity, DoubleTopic distance, DoubleTopic limelightTY, DoubleTopic heightDiff) {
        // Check constants.java file to see the values provided
        flywheelSlot0Configs.kS = ShooterConstants.flywheel_kS;
        flywheelSlot0Configs.kV = ShooterConstants.flywheel_kV;
        flywheelSlot0Configs.kP = ShooterConstants.flywheel_kP;
        flywheelSlot0Configs.kI = ShooterConstants.flywheel_kI;
        flywheelSlot0Configs.kD = ShooterConstants.flywheel_kD;

        m_leftFlywheelLead.getConfigurator().apply(flywheelSlot0Configs);
        m_leftFlywheelFollow.getConfigurator().apply(flywheelSlot0Configs);
        m_rightFlywheelLead.getConfigurator().apply(flywheelSlot0Configs);
        m_rightFlywheelFollow.getConfigurator().apply(flywheelSlot0Configs);

        m_rightFlywheelLead.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        m_rightFlywheelFeeder.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_leftFlywheelLead.setNeutralMode(NeutralModeValue.Coast);
        m_leftFlywheelFollow.setNeutralMode(NeutralModeValue.Coast);
        m_rightFlywheelLead.setNeutralMode(NeutralModeValue.Coast);
        m_rightFlywheelFollow.setNeutralMode(NeutralModeValue.Coast);

        m_leftFlywheelFeeder.setNeutralMode(NeutralModeValue.Brake);
        m_rightFlywheelFeeder.setNeutralMode(NeutralModeValue.Brake);

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

        flywheelSpeedPub = flywheelVelocity.publish();
        distancePub = distance.publish();
        limelightTYPub = limelightTY.publish();
        heightDiffPub = heightDiff.publish();
    }

    public void setFlywheelSpeed(double rps) {
        m_rightFlywheelLead.setControl(m_VelocityVoltageRequest.withVelocity(rps));
        m_leftFlywheelLead.setControl(m_VelocityVoltageRequest.withVelocity(rps));
    }

    public void setFeederSpeed(double rps) {
        m_leftFlywheelFeeder.setControl(m_VelocityVoltageRequest.withVelocity(rps));
        m_rightFlywheelFeeder.setControl(m_VelocityVoltageRequest.withVelocity(rps));
    }

    public void stopFlywheels() {
        m_leftFlywheelLead.stopMotor();
        m_rightFlywheelLead.stopMotor();
    }

    public void stopFeeder() {
        m_leftFlywheelFeeder.stopMotor();
        m_rightFlywheelFeeder.stopMotor();
    }

    public void stopSystem() {
        stopFlywheels();
        stopFeeder();
    } 

    public boolean flywheelAtVelocity(double rps, double tolerance) {
        return m_rightFlywheelLead.getVelocity().isNear(rps, tolerance)
            && m_leftFlywheelLead.getVelocity().isNear(rps, tolerance);
    }

    public boolean ready() {
        return flywheelAtVelocity(0, 0);
    }

    public Command shootSequence() {
        return 
            this.runOnce(() -> setFlywheelSpeed(0))
            .until(this::ready)
            .andThen(runOnce(() -> setFeederSpeed(0)))
            .finallyDo(this::stopSystem);
    }

    public Command shootPauseSequence() {
        return new ParallelCommandGroup(
            this.runOnce(() -> setFlywheelSpeed(50)),
            new ConditionalCommand(this.runOnce(() -> setFeederSpeed(30)), this.runOnce(() -> stopFeeder()), this::ready).repeatedly()
        ).finallyDo(() -> this.stopSystem());
    }

    public Command spinFeeder() {
        return this.runEnd(
            () -> {
                // set velocity to 8 rps, add 0.5 V to overcome gravity
                m_rightFlywheelFeeder.setControl(m_velocityTorqueRequest.withVelocity(50));
                m_leftFlywheelFeeder.setControl(m_velocityTorqueRequest.withVelocity(50));
            },
            () -> {
                m_leftFlywheelFeeder.stopMotor();
                m_rightFlywheelFeeder.stopMotor();
            });
    }

    @Override
    public void periodic() {
        flywheelSpeedPub.set(m_leftFlywheelLead.getVelocity().getValueAsDouble());
        double ty = LimelightHelpers.getTY("limelight");
        limelightTYPub.set(ty);
        double limelightSightHeight = goalHeightIn - limelightHeightIn;
        heightDiffPub.set(limelightSightHeight);
        double dist = limelightSightHeight / Math.tan((limelightDeg + ty) * Math.PI / 180);
        distancePub.set(dist);
    }
}
