// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
    private CANSparkMax m_turretMotor;
    private SparkPIDController m_turretPID;
    private RelativeEncoder m_relEncoder;
    private AbsoluteEncoder m_absEncoder;

    // Max Clockwise (CW) and Max Counter Clockwise positions.
    private final double MAX_CW_POS;
    private final double MAX_CCW_POS;

    private PIDController m_pidController;
    private double target_position;

    /** Creates a new TurretSubsystem. */
    public TurretSubsystem() {
        /* Systems */
        m_turretMotor = new CANSparkMax(31, MotorType.kBrushless);
        m_turretMotor.restoreFactoryDefaults();
        m_turretMotor.setIdleMode(IdleMode.kCoast);
        m_turretMotor.setInverted(true);

        this.MAX_CCW_POS = -35;
        this.MAX_CW_POS = 200;
        this.target_position = 0;

        this.m_pidController = new PIDController(.1, 0.001, 0.0);
        this.m_pidController.setTolerance(.5);

        m_relEncoder = m_turretMotor.getEncoder();
        m_absEncoder = m_turretMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        m_relEncoder.setPosition(104.000000004);
    }

    public void set(double speed) {
        this.m_turretMotor.set(speed);
    }

    public Command setTargetPosition(double target_deg) {
        double target_pos = target_deg * 0.5777777778;

        if (target_deg <= MAX_CW_POS && target_deg >= MAX_CCW_POS) {
            return new FunctionalCommand(() -> {
            },
                    () -> {
                        double effort = this.m_pidController.calculate(this.m_turretMotor.getEncoder().getPosition(),
                                target_pos);
                        Logger.recordOutput("Turret/Effort", effort);
                        this.m_turretMotor.set(effort);
                    },
                    (i) -> {
                        this.m_turretMotor.set(0);
                    },
                    () -> this.m_pidController.atSetpoint(), this);
        }

        return Commands.none();
    }

    public void setTargetHeading(double target_deg) {
        double target_pos = target_deg * (208 / 360);

        if (target_deg <= MAX_CW_POS && target_deg >= MAX_CCW_POS) {
            this.target_position = target_pos;
        }
    }

    public Command goToHeading(double target_deg) {
        return runOnce(() -> setTargetPosition(target_deg));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Turret/Current Pos", this.m_relEncoder.getPosition());
        Logger.recordOutput("Turret/Current Absolute Pos", this.m_absEncoder.getPosition());
        Logger.recordOutput("Turret/Target Pos", this.m_pidController.getSetpoint());
        Logger.recordOutput("Turret/At Setpoint", this.m_pidController.atSetpoint());
    }

    public void stop() {
        m_turretMotor.stopMotor(); // try stopMotor()
    }
}
