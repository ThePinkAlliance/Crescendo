package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        //this.m_relEncoder.setPosition(0);
        this.m_motor.setIdleMode(IdleMode.kCoast);

        var m_pidController = m_motor.getPIDController();

        double kP = 0.1;
        double kI = 1e-5;

        m_pidController.setP(kP);
        m_pidController.setI(kI);

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

    public void setAngle(double angle) {
        double targetRotations = angle * (57.2 / (58.2 + 2));

        this.m_motor.getPIDController().setReference(targetRotations, ControlType.kPosition);
        this.currentTargetRotations = targetRotations;
    }

    public void resetAngle() {
        this.m_relEncoder.setPosition(0);
    }

}
