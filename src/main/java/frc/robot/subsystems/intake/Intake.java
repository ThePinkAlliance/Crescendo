// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants.RobotType;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private CollectorEncoderv2 m_encoder;

    private final CANSparkMax angleSparkMax;
    private final CANSparkFlex collectMotor;
    private DigitalInput m_noteSwitch;

    public final double angleFF;
    public final double CANCODER_ROTATIONS_TO_MOTOR_TICKS;
    public final PIDController anglePidController;

    public Intake() {

        this.angleSparkMax = new CANSparkMax(22, MotorType.kBrushless);
        this.collectMotor = new CANSparkFlex(21, MotorType.kBrushless);
        this.m_noteSwitch = new DigitalInput(9);

        this.m_encoder = new CollectorEncoderv2();
        this.collectMotor.setIdleMode(IdleMode.kBrake);
        this.angleSparkMax.setIdleMode(IdleMode.kBrake);
        this.angleSparkMax.setInverted(true);

        this.anglePidController = new PIDController(1, 0.2, 0);
        this.anglePidController.setTolerance(2);
        this.angleFF = 0;
        this.CANCODER_ROTATIONS_TO_MOTOR_TICKS = 0.0268456376;
    }

    public boolean noteFound() {
        return !m_noteSwitch.get();
    }

    public BooleanSupplier noteFoundSupplier() {
        return () -> !m_noteSwitch.get();
    }

    private void moveCollector(double speed) {
        this.angleSparkMax.set(speed);
    }

    public Command collectUntilFound(double power) {
        return new FunctionalCommand(() -> {
        },
                () -> this.collectMotor.set(power),
                (interrupted) -> this.collectMotor.set(0),
                () -> noteFound(),
                this);
    }

    public Command setAnglePosition(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    /**
                     * Calculate the desired control effort using WPIlib pid controller and
                     * calculate feedforward using angleFF * Math.sin(rotation_ratio_collector)
                     */

                    double applied_ff = angleFF * Math.sin(m_encoder.getPosition() * CANCODER_ROTATIONS_TO_MOTOR_TICKS);
                    double effort = anglePidController.calculate(m_encoder.getPosition(),
                            pos);

                    Logger.recordOutput("Intake/Control Effort 2", effort);

                    // scale control effort to a ratio to make it useable with voltage control.
                    effort = (effort * CANCODER_ROTATIONS_TO_MOTOR_TICKS) + applied_ff;

                    Logger.recordOutput("Intake/Control Effort", effort);
                    Logger.recordOutput("Intake/Control Error", anglePidController.getPositionError());
                    Logger.recordOutput("Intake/Setpoint", pos);
                    Logger.recordOutput("Intake/FF", applied_ff);

                    this.angleSparkMax.setVoltage(effort * 12);
                },
                (interrupt) -> {
                    this.angleSparkMax.set(0);
                },
                () -> this.anglePidController.atSetpoint(), this);
    }

    public void stop() {
        this.collectMotor.set(0);
    }

    public double getControlError() {
        return this.anglePidController.getPositionError();
    }

    public Command stowCollector() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(-0.30),
                (interrupted) -> this.moveCollector(0.0),
                () -> isStowed(),
                this);
    }

    public Command deployCollector() {
        return deployCollector(0.25);
    }

    public Command deployCollector(double speed) {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(speed),
                (interrupted) -> this.moveCollector(0.0),
                () -> isDeployed(),
                this);
    }

    public Command transferNote() {
        return new FunctionalCommand(() -> {
            this.setAnglePosition(367);
        }, () -> {
        }, (i) -> {
            this.collectMotor.set(0.85);
        }, () -> canDeliver(), this);
    }

    public Command goToTransfer() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(-0.25),
                (interrupted) -> {
                    this.moveCollector(0.0);
                    this.collectMotor.set(0.85);
                },
                () -> canDeliver(),
                this);
    }

    public boolean isStowed() {
        boolean value = false;
        // if (m_encoder.getPosition() < 10.0) {
        // value = true;
        // }
        return value;
    }

    public boolean isDeployed() {
        boolean value = false;
        if (m_encoder.getPosition() > 700) {
            value = true;
        }
        return value;
    }

    public boolean canDeliver() {
        boolean value = false;
        if (Constants.RobotConstants.CURRENT_ROBOT == RobotType.ROBOT_ONE) {
            if (m_encoder.getPosition() > 242 && m_encoder.getPosition() < 312 * 1.5) {
                value = true;
            }
        } else {
            // 21.21 ideal; 24.7 bottom; 16.8 top;
            if (m_encoder.getPosition() <= 21.3 && m_encoder.getPosition() >= 17.5) {
                value = true;
            }
        }
        Logger.recordOutput("Intake/Hex Encoder", m_encoder.getPosition());
        return value;
    }

    public Command setCollectorPower(double speed) {
        return runOnce(() -> this.collectMotor.set(speed));
    }

    public void setCollectorPowerRaw(double power) {
        this.collectMotor.set(power);
    }

    public double getCollectorPosition() {
        return this.m_encoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/Sensor Far", noteFound());
        Logger.recordOutput("Intake/Angle Position",
                this.m_encoder.getPosition());
        Logger.recordOutput("Intake/Collect Raw Encoder", this.m_encoder.getRawPosition());
        Logger.recordOutput("Intake/Collect Velocity", this.collectMotor.getEncoder().getVelocity());
    }
}
