package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

public class Angle extends SubsystemBase {

    private CANSparkMax m_motor;
    private CANcoder m_angleCancoder;
    private RelativeEncoder m_relEncoder;
    private double target_rotations;

    public Angle() {
        this.m_motor = new CANSparkMax(41, MotorType.kBrushless);
        this.m_angleCancoder = new CANcoder(20);
        this.target_rotations = 0;

        var cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfig.MagnetSensor.MagnetOffset = -0.158;

        this.m_angleCancoder.getConfigurator().apply(cancoderConfig);

        this.m_motor.restoreFactoryDefaults();
        this.m_relEncoder = m_motor.getEncoder();
        this.m_motor.setIdleMode(IdleMode.kCoast);
        this.m_motor.setInverted(true);

        var m_pidController = m_motor.getPIDController();

        double kP = 0.09;
        double kI = 0.00;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setFF(.001);
        m_pidController.setOutputRange(-1, 1);
    }

    @Override
    public void periodic() {
        var fowardSwitch = m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        double relativeEncoderPosition = m_relEncoder.getPosition();

        Logger.recordOutput("Shooter/Angle Position", relativeEncoderPosition);
        Logger.recordOutput("Shooter/Angle Real", (relativeEncoderPosition) * (52.2 / 53.95));
        Logger.recordOutput("Shooter/CANCoder Angle", this.m_angleCancoder.getAbsolutePosition().getValueAsDouble());
        Logger.recordOutput("Shooter/CANcoder Angle Real",
                getCancoderAngle());

        Logger.recordOutput("Shooter/Foward Switch", fowardSwitch.isPressed());
    }

    public void disable() {
        this.m_motor.set(0);
    }

    public void updateTuneAnglePID() {
        double targetRotation = SmartDashboard.getNumber("Angle Target", 0);

        this.setAngle(targetRotation);
    }

    public double getCancoderAngle() {
        return ((m_angleCancoder.getAbsolutePosition().getValueAsDouble()) * 360) + 1;
    }

    public void setAngleNew(double angle) {
        double rotationDiff = (angle - getCancoderAngle()) * (52.07 / 51.71);
        double desired_rotations = this.m_motor.getEncoder().getPosition() + rotationDiff;

        this.m_motor.getPIDController().setReference(desired_rotations,
                ControlType.kPosition);
        this.target_rotations = desired_rotations;
        Logger.recordOutput("Shooter/Angle Setpoint", desired_rotations);
        Logger.recordOutput("Shooter/Angle Ref", angle);
    }

    public void setAngle(double angle) {
        double targetRotations = angle * (51.71 / 52.07);

        this.m_motor.getPIDController().setReference(targetRotations, ControlType.kPosition);
    }

    public void resetAngle() {
        this.m_relEncoder.setPosition(0);
    }

    public Command setAngleCommand(double angle) {
        double position = this.getCancoderAngle();

        if (position >= Constants.AngleConstants.MIN_ANGLE) {
            return runOnce(() -> this.setAngleNew(angle));
        } else {
            return Commands.none();
        }
    }

    public double getControlError() {
        return Math.abs(this.target_rotations - this.m_relEncoder.getPosition());
    }

    public void stop() {
        this.m_motor.getPIDController().setReference(0, ControlType.kCurrent);
    }

    public double getSpeed() {
        return this.m_motor.get();
    }

    public Command setAngleCommandNew(double angle) {
        return runOnce(() -> this.setAngleNew(angle));
    }

    public Command GotoAngle(double setpoint) {
        return new FunctionalCommand(() -> this.setAngleNew(setpoint), () -> {
        }, (i) -> {
        }, () -> this.getControlError() <= 8 && this.getSpeed() <= 0.05, this);
    }
}
