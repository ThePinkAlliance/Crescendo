// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberR1 extends SubsystemBase {

    public enum ClimberSide {
        LEFT,
        RIGHT
    }

    private CANSparkMax m_leftMotor;
    private CANSparkMax m_rightMotor;

    private static final int LeftMotorDeviceID = 24;
    private static final int RightMotorDeviceID = 23;

    private SparkPIDController m_pidControllerL;
    private SparkPIDController m_pidControllerR;

    private RelativeEncoder m_relLeftEncoder;
    private RelativeEncoder m_relRightEncoder;

    // private AbsoluteEncoder m_absEncoder;

    public ClimberR1() {
        SmartDashboard.putBoolean("use Shuffleboard", false);
        SmartDashboard.putNumber("Left Target", 0);
        SmartDashboard.putNumber("Right Target", 0);

        m_leftMotor = new CANSparkMax(LeftMotorDeviceID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(RightMotorDeviceID, MotorType.kBrushless);

        setMotor(m_leftMotor, LeftMotorDeviceID);
        setMotor(m_rightMotor, RightMotorDeviceID);

        m_pidControllerL = setPID(m_leftMotor);
        m_pidControllerR = setPID(m_rightMotor);

        m_relLeftEncoder = m_leftMotor.getEncoder();
        m_relRightEncoder = m_rightMotor.getEncoder();

        // m_relLeftEncoder.setPosition(0);
        // m_relRightEncoder.setPosition(0);
    }

    public void setMotor(CANSparkMax m_motor, int DeviceID) {
        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public SparkPIDController setPID(CANSparkMax m_motor) {

        SparkPIDController m_pidController = m_motor.getPIDController();

        double kP = 0.1;
        double kI = 1e-5;
        double kD = 0.1;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);

        return m_pidController;

    }

    @Override
    public void periodic() {
        putShuffleboardValues();
        putShuffleboardControls();

        Logger.recordOutput("Climbers/Left Pos", this.m_leftMotor.getEncoder().getPosition());
        Logger.recordOutput("Climbers/Right Pos", this.m_rightMotor.getEncoder().getPosition());
    }

    public void putShuffleboardValues() {
        double m_relativeLeftMotorEncoderPosition = m_relLeftEncoder.getPosition();
        double m_relativeRightMotorEncoderPosition = m_relRightEncoder.getPosition();

        SmartDashboard.putNumber("L Motor Enc", m_relativeLeftMotorEncoderPosition);
        SmartDashboard.putNumber("R Motor Enc", m_relativeRightMotorEncoderPosition);
    }

    public void putShuffleboardControls() {
        boolean useShuffleboard = SmartDashboard.getBoolean("use Shuffleboard", false);

        if (useShuffleboard) {
            /* Shuffleboard controls */
            double leftTarget = SmartDashboard.getNumber("Left Target", 0);
            double rightTarget = SmartDashboard.getNumber("Right Target", 0);

            m_pidControllerL.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
            m_pidControllerR.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
        }

    }

    public Command resetRightClimber() {
        Timer w = new Timer();
        return new FunctionalCommand(
                () -> {
                    this.m_rightMotor.set(-.5);
                    w.start();
                },
                () -> {
                },
                (i) -> {
                    this.m_rightMotor.set(0);
                    this.m_rightMotor.getEncoder().setPosition(0);
                },
                () -> this.m_rightMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()
                        || w.hasElapsed(2),
                this);
    }

    public Command resetLeftClimber() {
        Timer w = new Timer();
        return new FunctionalCommand(
                () -> {
                    this.m_leftMotor.set(.5);
                    w.start();
                },
                () -> {
                },
                (i) -> {
                    this.m_leftMotor.set(0);
                    this.m_leftMotor.getEncoder().setPosition(0);
                },
                () -> this.m_leftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()
                        || w.hasElapsed(2),
                this);
    }

    public Command setRightClimberPos(double pos) {
        Timer w = new Timer();
        return new FunctionalCommand(
                () -> {
                    this.m_rightMotor.set(.8);
                    w.start();
                },
                () -> {
                },
                (i) -> {
                    this.m_rightMotor.set(0);
                },
                () -> Math.abs(pos - this.m_rightMotor.getEncoder().getPosition()) <= 4
                        || w.hasElapsed(2),
                this);
    }

    public Command setLeftTargetPosition(double leftMotorTargetRotation) {
        // double calculatedRotations = targetRotation * (57.2 / (58.2 + 2));
        return runOnce(
                () -> {
                    m_pidControllerL.setReference(leftMotorTargetRotation, CANSparkMax.ControlType.kPosition);
                });
    }

    public Command setRightTargetPosition(double rightMotorTargetRotation) {
        // double calculatedRotations = targetRotation * (57.2 / (58.2 + 2));
        return runOnce(
                () -> {
                    m_pidControllerR.setReference(rightMotorTargetRotation, CANSparkMax.ControlType.kPosition);
                });
    }

    public void setRightTargetPosition2(double rightMotorTargetRotation) {
        m_pidControllerR.setReference(rightMotorTargetRotation, CANSparkMax.ControlType.kPosition);
    }

    public void setLeftTargetPosition2(double leftMotorTargetRotation) {
        m_pidControllerL.setReference(leftMotorTargetRotation, CANSparkMax.ControlType.kPosition);
    }

    public void rightReset() {
        m_relRightEncoder.setPosition(0);
    }

    public void leftReset() {
        m_relLeftEncoder.setPosition(0);
    }
}