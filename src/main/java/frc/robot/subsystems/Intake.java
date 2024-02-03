// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    CANSparkMax m_intake;
    SparkPIDController m_pidControllerIntake;
    private RelativeEncoder m_encoderIntake;
    public double ikP, ikI, ikD, ikIz, ikFF, ikMaxOutput, ikMinOutput, imaxRPM;

    CANSparkMax m_angle;
    SparkPIDController m_pidControllerAngle;
    private RelativeEncoder m_encoderAngle;
    public double akP, akI, akD, akIz, akFF, akMaxOutput, akMinOutput, amaxRPM;
    private double currentTargetRotations;

    /** Creates a new Loader. */
    public Intake() {
        /**
         * Configure Intake Spark MAX.
         */

        m_intake = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);
        m_intake.restoreFactoryDefaults();
        m_intake.setIdleMode(IdleMode.kCoast);
        m_pidControllerIntake = m_intake.getPIDController();
        m_encoderIntake = m_intake.getEncoder();
        ikP = 6e-5;
        ikI = 0;
        ikD = 0;
        ikIz = 0;
        ikFF = 0.000015;
        ikMaxOutput = 1;
        ikMinOutput = -1;
        imaxRPM = 5700;

        // set PID coefficients
        m_pidControllerIntake.setP(ikP);
        m_pidControllerIntake.setI(ikI);
        m_pidControllerIntake.setD(ikD);
        m_pidControllerIntake.setIZone(ikIz);
        m_pidControllerIntake.setFF(ikFF);
        m_pidControllerIntake.setOutputRange(ikMinOutput, ikMaxOutput);

        /**
         * Configure Intake Angle Spark MAX.
         */

        m_angle = new CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless);
        m_angle.restoreFactoryDefaults();
        m_angle.setIdleMode(IdleMode.kCoast);
        m_pidControllerAngle = m_angle.getPIDController();
        m_encoderAngle = m_angle.getEncoder();
        akP = 6e-5;
        akI = 0;
        akD = 0;
        akIz = 0;
        akFF = 0.000015;
        akMaxOutput = 1;
        akMinOutput = -1;
        amaxRPM = 5700;

        // set PID coefficients
        m_pidControllerAngle.setP(akP);
        m_pidControllerAngle.setI(akI);
        m_pidControllerAngle.setD(akD);
        m_pidControllerAngle.setIZone(akIz);
        m_pidControllerAngle.setFF(akFF);
        m_pidControllerAngle.setOutputRange(akMinOutput, akMaxOutput);
        m_encoderAngle = m_angle.getEncoder();
        m_encoderAngle.setPosition(0);
        m_angle.setIdleMode(IdleMode.kCoast);

        setupAngleDashboard();
        setupIntakeDashboard();
    }

    private void setupIntakeDashboard() {

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("iSpark P Gain", ikP);
        SmartDashboard.putNumber("iSpark I Gain", ikI);
        SmartDashboard.putNumber("iSpark D Gain", ikD);
        SmartDashboard.putNumber("iSpark I Zone", ikIz);
        SmartDashboard.putNumber("iSpark Feed Forward", ikFF);
        SmartDashboard.putNumber("iSpark Max Output", ikMaxOutput);
        SmartDashboard.putNumber("iSpark Min Output", ikMinOutput);
    }

    private void setupAngleDashboard() {
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("aSpark P Gain", akP);
        SmartDashboard.putNumber("aSpark I Gain", akI);
        SmartDashboard.putNumber("aSpark D Gain", akD);
        SmartDashboard.putNumber("aSpark I Zone", akIz);
        SmartDashboard.putNumber("aSpark Feed Forward", akFF);
        SmartDashboard.putNumber("aSpark Max Output", akMaxOutput);
        SmartDashboard.putNumber("aSpark Min Output", akMinOutput);
        SmartDashboard.putNumber("aRealative Angle", 0);
        SmartDashboard.putNumber("aAngle Target", 0);
        SmartDashboard.putNumber("aPoint error", 0);
    }

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
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
