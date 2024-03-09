// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final CANSparkMax angleSparkMax;
    //private final CANSparkMax collectSparkFlex;
    private final CANSparkFlex collectSparkFlex;

    private final SparkPIDController collectPIDController;
    private final RelativeEncoder collectEncoder;

    private Encoder hexEncoder;
    private DigitalInput m_noteSwitch;
    public static final int[] HEX_ENCODER_IDS = { 4, 5 };
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double PULSE_PER_REVOLUTION = 250; // Need to revisit this value!!
    public final double DISTANCE_PER_PULSE = (double) (Math.PI * WHEEL_DIAMETER) / PULSE_PER_REVOLUTION;
    public final double angleFF;
    public final PIDController anglePidController;

    public Intake() {
        this.angleSparkMax = new CANSparkMax(22, MotorType.kBrushless);
        //this.collectSparkFlex = new CANSparkMax(21, MotorType.kBrushless);
        this.collectSparkFlex = new CANSparkFlex(21, MotorType.kBrushless);
        this.m_noteSwitch = new DigitalInput(9);

        this.hexEncoder = new Encoder(HEX_ENCODER_IDS[0], HEX_ENCODER_IDS[1]);
        this.collectEncoder = collectSparkFlex.getEncoder();

        this.collectSparkFlex.setIdleMode(IdleMode.kBrake);

        this.angleSparkMax.setIdleMode(IdleMode.kBrake);
        this.angleSparkMax.setInverted(true);

        this.collectPIDController = collectSparkFlex.getPIDController();

        this.collectPIDController.setP(.1);

        this.anglePidController = new PIDController(.5, 0.02, 0.0);
        this.anglePidController.setTolerance(15);
        this.angleSparkMax.getPIDController().setP(.1);

        this.angleFF = 0.2;

        SetupHexEncoder(hexEncoder, true);

        SmartDashboard.putNumber("Vortex Target", 0);
        //SmartDashboard.putBoolean("set Vortex Speed", false);
        SmartDashboard.putBoolean("stop Vortex", true);

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

    public boolean noteFound() {
        return !m_noteSwitch.get();
    }

    private void moveCollector(double speed) {
        this.angleSparkMax.set(speed);
    }

    public Command collectUntilFound(double power) {
        return new FunctionalCommand(() -> {
        },
                () -> this.collectSparkFlex.set(power),
                (interrupted) -> this.collectSparkFlex.set(0),
                () -> noteFound(),
                this);
    }

    public Command stopWhenFound(double power) {
        return new FunctionalCommand(() -> {
        },
                () -> {
                },
                (interrupted) -> this.collectSparkFlex.set(0),
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

                    double applied_ff = angleFF * Math.sin(hexEncoder.get() * (1 / 734));
                    double effort = anglePidController.calculate(hexEncoder.get(),
                            pos);

                    Logger.recordOutput("Intake/Control Effort 2", effort);

                    // scale control effort to a ratio to make it useable with voltage control.
                    effort = (effort * 0.0013623978) + applied_ff;

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

    public Command setAnglePosition(double pos, double speed) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    /**
                     * Calculate the desired control effort using WPIlib pid controller and
                     * calculate feedforward using angleFF * Math.cos(rotation_ratio_collector)
                     */

                    double applied_ff = angleFF * Math.sin(hexEncoder.get() * (1 / 734));
                    double effort = anglePidController.calculate(hexEncoder.get(),
                            pos);

                    Logger.recordOutput("Intake/Control Effort 2", effort);

                    // scale control effort to a ratio to make it useable with voltage control.
                    effort = (effort * 0.0013623978) + applied_ff;

                    Logger.recordOutput("Intake/Control Effort", effort);
                    Logger.recordOutput("Intake/Control Error", anglePidController.getPositionError());
                    Logger.recordOutput("Intake/Setpoint", pos);
                    Logger.recordOutput("Intake/FF", applied_ff);

                    this.collectSparkFlex.set(speed);
                    this.angleSparkMax.setVoltage(effort * 12);
                },
                (interrupt) -> {
                    this.angleSparkMax.set(0);
                },
                () -> this.anglePidController.atSetpoint(), this);
    }

    public DoubleSupplier getAngleSupplier() {
        return () -> this.hexEncoder.get();
    }

    // Untested
    public Command setAnglePositionHold(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    /**
                     * Calculate the desired control effort using WPIlib pid controller and
                     * calculate feedforward using angleFF * Math.cos(rotation_ratio_collector)
                     */

                    double applied_ff = angleFF * Math.sin(hexEncoder.get() * (1 / 734));
                    double effort = anglePidController.calculate(hexEncoder.get(),
                            pos);

                    Logger.recordOutput("Intake/Control Effort 2", effort);

                    // scale control effort to a ratio to make it useable with voltage control.
                    effort = (effort * 0.0013623978) + applied_ff;

                    Logger.recordOutput("Intake/Control Effort", effort);
                    Logger.recordOutput("Intake/Control Error", anglePidController.getPositionError());
                    Logger.recordOutput("Intake/Setpoint", pos);
                    Logger.recordOutput("Intake/FF", applied_ff);

                    this.angleSparkMax.setVoltage(effort * 12);
                },
                (interrupt) -> {
                    double applied_ff = angleFF * Math.sin(hexEncoder.get() * (1 / 734));

                    this.angleSparkMax.set(applied_ff);
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

    public Command ampShot(){
        Timer timer = new Timer();
        return new FunctionalCommand(() -> {timer.reset(); timer.start();
        },
                () ->  this.collectSparkFlex.set(-1),
                (interrupted) -> this.stowCollector(),
                () -> timer.hasElapsed(2),
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
            this.collectSparkFlex.set(0.85);
        }, () -> canDeliver(), this);
    }

    public Command goToTransfer() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(-0.25),
                (interrupted) -> {
                    this.moveCollector(0.0);
                    this.collectSparkFlex.set(0.85);
                },
                () -> canDeliver(),
                this);
    }

    public boolean isStowed() {
        boolean value = false;
        if (hexEncoder.get() < 10.0) {
            value = true;
        }
        return value;
    }

    public boolean isDeployed() {
        boolean value = false;
        if (hexEncoder.get() > 700) {
            value = true;
        }
        return value;
    }

    public boolean canDeliver() {
        boolean value = false;
        if (hexEncoder.get() > 242 && hexEncoder.get() < 312 * 1.5) {
            value = true;
        }
        Logger.recordOutput("Intake/Hex Encoder", hexEncoder.get());
        return value;
    }

    public Command setCollectorPower(double speed) {
        return runOnce(() -> this.collectSparkFlex.set(speed));
    }

    @Override
    public void periodic() {

        double vortexTestSpeedAsRPM = SmartDashboard.getNumber("Vortex Target", 0);
        //boolean setVortexSpeed = SmartDashboard.getBoolean("set Vortex Speed", false);
        boolean vortexStopped = SmartDashboard.getBoolean("stop Vortex", true);


        if (!vortexStopped) {
            //collectPIDController.setReference(vortexTestSpeedAsRPM, CANSparkBase.ControlType.kVelocity);
            collectSparkFlex.set(vortexTestSpeedAsRPM);
        } else {
            collectSparkFlex.set(0);
        }

        double currentVortexSpeed = collectEncoder.getVelocity();
        SmartDashboard.putNumber("Vortex Speed", currentVortexSpeed);

        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/Sensor Far", noteFound());
        Logger.recordOutput("Intake/Angle Position",
                hexEncoder.get());
        Logger.recordOutput("Intake/Collect Velocity", this.collectEncoder.getVelocity());
    }
}
