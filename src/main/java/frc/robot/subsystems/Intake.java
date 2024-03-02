// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants.RobotType;

import java.lang.module.ResolutionException;

import javax.swing.text.Position;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    class CurrentEncoder {

        private Encoder hexEncoder;
        private CANcoder canCoder;

        private RobotType currentRobot = Constants.RobotConstants.CURRENT_ROBOT;

        public CurrentEncoder() {
            if (currentRobot == RobotType.ROBOT_ONE) {
                this.hexEncoder = new Encoder(HEX_ENCODER_IDS[0], HEX_ENCODER_IDS[1]);
                SetupHexEncoder(hexEncoder, true);

            } else if (currentRobot == RobotType.ROBOT_TWO) {
                this.canCoder = new CANcoder(7, "rio");
                var cancoderConfig = new CANcoderConfiguration();
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
                cancoderConfig.MagnetSensor.MagnetOffset = 0.00317;
                this.canCoder.getConfigurator().apply(cancoderConfig);

            }
        }

        public double getPosition() {

            double position;

            if (currentRobot == RobotType.ROBOT_ONE) {
                position = this.hexEncoder.get();
            } else {
                position = this.canCoder.getAbsolutePosition().getValueAsDouble();
            }

            return position;
        }

        public void reset() {
            if (currentRobot == RobotType.ROBOT_ONE) {
                this.hexEncoder.reset();
            } else {
                this.canCoder.setPosition(0);
            }
        }

        private void SetupHexEncoder(Encoder enc, boolean reverseDirection) {

            if (enc == null)
                return;
            enc.setMaxPeriod(.1);
            enc.setMinRate(10);
            System.out.println("SetupHexEncoder: Distance per Pulse: " + DISTANCE_PER_PULSE);
            enc.setDistancePerPulse(DISTANCE_PER_PULSE);
            enc.setReverseDirection(reverseDirection);
            enc.setSamplesToAverage(7);
            enc.reset();
        }

    };

    private CurrentEncoder m_encoder;

    private final CANSparkMax angleSparkMax;
    private final TalonFX collectMotor;

    private DigitalInput m_noteSwitch;
    public static final int[] HEX_ENCODER_IDS = { 4, 5 };
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double PULSE_PER_REVOLUTION = 250; // Need to revisit this value!!
    public final double DISTANCE_PER_PULSE = (double) (Math.PI * WHEEL_DIAMETER) / PULSE_PER_REVOLUTION;
    public final double angleFF;
    public final PIDController anglePidController;

    public Intake() {

        this.angleSparkMax = new CANSparkMax(22, MotorType.kBrushless);
        this.collectMotor = new TalonFX(21);
        this.m_noteSwitch = new DigitalInput(9);

        this.m_encoder = new CurrentEncoder();
        this.collectMotor.setNeutralMode(NeutralModeValue.Brake);
        this.angleSparkMax.setIdleMode(IdleMode.kCoast);
        this.angleSparkMax.setInverted(true);

        this.anglePidController = new PIDController(0, 0, 0);
        this.angleSparkMax.getPIDController().setP(.1);
        this.angleFF = 0.1;

        this.m_encoder.reset();

    }

    public boolean noteFound() {
        return !m_noteSwitch.get();
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
                     * calculate feedforward using angleFF * Math.cos(rotation_ratio_collector)
                     */

                    double effort = this.anglePidController.calculate(m_encoder.getPosition(),
                            pos)
                            + (angleFF * Math.sin(m_encoder.getPosition() * (1 / 734)));

                    Logger.recordOutput("Intake/Control Effort 2", effort);

                    // scale control effort to a ratio to make it useable with voltage control.
                    double full_error = Math.abs(pos - m_encoder.getPosition());
                    effort = effort * (1 / full_error);

                    Logger.recordOutput("Intake/Control Effort", effort);
                    Logger.recordOutput("Intake/Full Error", full_error);
                    Logger.recordOutput("Intake/Setpoint", pos);
                    Logger.recordOutput("Intake/FF", (angleFF * Math.cos(m_encoder.getPosition() * (1 /
                            734))));

                    this.angleSparkMax.setVoltage(effort * 12);
                },
                (interrupt) -> {
                    this.angleSparkMax.set(0);
                },
                () -> this.anglePidController.atSetpoint(), this);
    }

    public Command stowCollector() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(-0.20),
                (interrupted) -> this.moveCollector(0.0),
                () -> isStowed(),
                this);
    }

    public Command deployCollector() {
        return deployCollector(0.20);
    }

    public Command deployCollector(double speed) {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(speed),
                (interrupted) -> this.moveCollector(0.0),
                () -> isDeployed(),
                this);
    }

    public Command goToTransfer() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(-0.15),
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
        if (m_encoder.getPosition() > 242 && m_encoder.getPosition() < 312 * 1.5) {
            value = true;
        }
        Logger.recordOutput("Intake/Hex Encoder", m_encoder.getPosition());
        return value;
    }

    public Command setCollectorPower(double speed) {
        return runOnce(() -> this.collectMotor.set(speed));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/Sensor Far", noteFound());
        Logger.recordOutput("Intake/Angle Position",
                this.m_encoder.getPosition());
        Logger.recordOutput("Intake/Collect Velocity", this.collectMotor.getVelocity().getValueAsDouble());
    }
}
