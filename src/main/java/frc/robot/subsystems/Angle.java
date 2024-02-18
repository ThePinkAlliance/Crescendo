package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

public class Angle extends SubsystemBase {

    private CANSparkMax m_motor;
    private RelativeEncoder m_relEncoder;

    private double currentTargetRotations;

    public Angle() {
        this.m_motor = new CANSparkMax(41, MotorType.kBrushless);

        this.m_motor.restoreFactoryDefaults();
        this.m_relEncoder = m_motor.getEncoder();
        // this.m_relEncoder.setPosition(0);
        this.m_motor.setIdleMode(IdleMode.kCoast);

        var m_pidController = m_motor.getPIDController();

        double kP = 0.1;
        double kI = 1e-5;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setOutputRange(-0.5, 0.5);

        this.currentTargetRotations = 0;

        SmartDashboard.putNumber("point error", 0);
        SmartDashboard.putNumber("Realative Angle", 0);
        SmartDashboard.putNumber("Angle Target", 0);
    }

    @Override
    public void periodic() {
        var fowardSwitch = m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        double relativeEncoderPosition = m_relEncoder.getPosition();
        double angleError = currentTargetRotations - relativeEncoderPosition;

        Logger.recordOutput("Shooter/Angle Position", relativeEncoderPosition);
        Logger.recordOutput("Shooter/Angle Real", (relativeEncoderPosition) * (52.2 / 53.95));

        SmartDashboard.putNumber("Angle Error", angleError);
        SmartDashboard.putBoolean("Foward Switch", fowardSwitch.isPressed());
    }

    public void setPower(double power) {
        this.m_motor.set(power);
    }

    public void setAnglePID() {
        double targetRotation = SmartDashboard.getNumber("Angle Target", 0);

        this.setAngle(targetRotation);
    }

    public void setAngleNew(double angle) {
        double rotations = angle * (53.95 / 52.2);

        this.m_motor.getPIDController().setReference(rotations, ControlType.kPosition);
    }

    public void setAngle(double angle) {
        double targetRotations = angle * (57.2 / (58.2 + 2));

        this.m_motor.getPIDController().setReference(targetRotations, ControlType.kPosition);
        this.currentTargetRotations = targetRotations;
    }

    public void resetAngle() {
        this.m_relEncoder.setPosition(0);
    }

    public Command setAngleCommand(double angle) {
        return runOnce(() -> this.setAngle(angle));
    }

}
