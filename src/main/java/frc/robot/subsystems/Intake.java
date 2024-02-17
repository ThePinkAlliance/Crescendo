// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax angleSparkMax;
    private final CANSparkMax collectSparkMax;

<<<<<<< Updated upstream
    CANSparkMax m_angle;
    SparkPIDController m_pidControllerAngle;
    private RelativeEncoder m_encoderAngle;
    public double akP, akI, akD, akIz, akFF, akMaxOutput, akMinOutput, amaxRPM;
    private double currentTargetRotations;
=======
    private final SparkPIDController anglePIDController;
    private final SparkPIDController collectPIDController;

    private final RelativeEncoder angleEncoder;
    private final RelativeEncoder collectEncoder;
>>>>>>> Stashed changes

    public Intake() {
        this.angleSparkMax = new CANSparkMax(22, MotorType.kBrushless);
        this.collectSparkMax = new CANSparkMax(21, MotorType.kBrushless);

        this.collectEncoder = collectSparkMax.getEncoder();
        this.angleEncoder = angleSparkMax.getEncoder();

        this.collectSparkMax.setIdleMode(IdleMode.kBrake);
        this.angleSparkMax.setIdleMode(IdleMode.kCoast);

        this.anglePIDController = angleSparkMax.getPIDController();
        this.anglePIDController.setP(.1);
        this.anglePIDController.setOutputRange(-.2, .2);

        this.collectPIDController = collectSparkMax.getPIDController();
        this.collectPIDController.setP(.1);
        this.collectPIDController.setFF(0);

        this.angleEncoder.setPosition(0);
    }

    public Command setCollectorAngle(double desiredAngle) {
        SmartDashboard.putNumber("collect_angle_setpoint", desiredAngle);

        return runOnce(() -> this.anglePIDController.setReference(desiredAngle, ControlType.kPosition));
    }

    public Command setCollectorSpeed(double desiredVelocity) {
        SmartDashboard.putNumber("collect_velocity_setpoint", desiredVelocity);

        return runOnce(() -> this.collectPIDController.setReference(desiredVelocity, ControlType.kVelocity));
    }

<<<<<<< Updated upstream
    public void moveIntake(double speed) {
        m_intake.set(speed);
    }

    public void stopIntake() {
        m_intake.set(0);
    }

    public void moveAngle(double rpms) {
        m_pidControllerAngle.setReference(rpms, CANSparkMax.ControlType.kVelocity);
    }

    public void stopAngle() {
        m_pidControllerAngle.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    public void setAnglePID() {

        double targetRotation = SmartDashboard.getNumber("aAngle Target", 0);

        this.setAngle(targetRotation);

    }

    public void setAngle(double angle) {
        double targetRotations = angle * (57.2 / (58.2 + 2));

        this.m_angle.getPIDController().setReference(targetRotations, ControlType.kPosition);
        this.currentTargetRotations = targetRotations;
=======
    public Command setCollectorSpeedP(double speed) {
        // SmartDashboard.putNumber("collect_velocity_setpoint", desiredVelocity);

        return runOnce(() -> this.collectSparkMax.set(speed));
>>>>>>> Stashed changes
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("collector_angle", this.angleEncoder.getPosition());
        SmartDashboard.putNumber("collect_velocity", this.collectEncoder.getVelocity());
        SmartDashboard.putNumber("collector_angle_absolute",
                this.angleSparkMax.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    }
}
