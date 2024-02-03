// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {

    CANSparkMax m_motor;
    SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    /** Creates a new Loader. */
    public Loader() {

        m_motor = new CANSparkMax(44, CANSparkLowLevel.MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kCoast);
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        setupDashboardInputs();
    }

    public void setupDashboardInputs() {

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Spark P Gain", kP);
        SmartDashboard.putNumber("Spark I Gain", kI);
        SmartDashboard.putNumber("Spark D Gain", kD);
        SmartDashboard.putNumber("Spark I Zone", kIz);
        SmartDashboard.putNumber("Spark Feed Forward", kFF);
        SmartDashboard.putNumber("Spark Max Output", kMaxOutput);
        SmartDashboard.putNumber("Spark Min Output", kMinOutput);
    }

    public void move(double rpms) {
        m_pidController.setReference(rpms, CANSparkMax.ControlType.kVelocity);
    }

    public void launch(double rpms) {
        move(-rpms);
    }

    public void load(double rpms) {
        move(rpms);
    }

    public void stop() {
        m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
