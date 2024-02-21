// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final CANSparkMax angleSparkMax;
    private final CANSparkMax collectSparkMax;

    // private final SparkPIDController anglePIDController;
    private final SparkPIDController collectPIDController;

    // private final RelativeEncoder angleEncoder;
    private final RelativeEncoder collectEncoder;

    //private final AnalogInput noteSwitchFar;
    //private final AnalogInput noteSwitchNear;
    private Encoder hexEncoder;
    private DigitalInput m_noteSwitch;
    public static final int[] HEX_ENCODER_IDS = { 4, 5 };
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double PULSE_PER_REVOLUTION = 250; // Need to revisit this value!!
    public final double DISTANCE_PER_PULSE = (double) (Math.PI * WHEEL_DIAMETER) / PULSE_PER_REVOLUTION;

    public Intake() {
        this.angleSparkMax = new CANSparkMax(22, MotorType.kBrushless);
        this.collectSparkMax = new CANSparkMax(21, MotorType.kBrushless);
        this.m_noteSwitch = new DigitalInput(9);

        this.hexEncoder = new Encoder(HEX_ENCODER_IDS[0], HEX_ENCODER_IDS[1]);
        this.collectEncoder = collectSparkMax.getEncoder();
       
        this.collectSparkMax.setIdleMode(IdleMode.kBrake);
        this.angleSparkMax.setIdleMode(IdleMode.kBrake);

        this.collectPIDController = collectSparkMax.getPIDController();
        this.collectPIDController.setP(.1);
        this.collectPIDController.setFF(0);

        SetupHexEncoder(hexEncoder, true);
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
        return m_noteSwitch.get();
    }

    private void moveCollector(double speed) {
        this.angleSparkMax.set(speed);
    }

    public Command setCollectorSpeed(double desiredVelocity) {
        SmartDashboard.putNumber("collect_velocity_setpoint", desiredVelocity);
        return new FunctionalCommand(() -> {
        },
                () -> this.collectSparkMax.set(desiredVelocity),
                (interrupted) -> this.collectSparkMax.set(0),
                () -> noteFound(),
                this);
    }

    public Command stowCollector() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(0.15),
                (interrupted) -> this.moveCollector(0.0),
                () -> isStowed(),
                this);
    }

    public Command deployCollector() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(-0.10),
                (interrupted) -> this.moveCollector(0.0),
                () -> isDeployed(),
                this);
    }

    public Command goToTransfer() {
        return new FunctionalCommand(() -> {
        },
                () -> this.moveCollector(0.15),
                (interrupted) -> {
                    this.moveCollector(0.0);
                    this.collectSparkMax.set(0.85);
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
        if (hexEncoder.get() > 680) {
            value = true;
        }
        return value;
    }

    public boolean canDeliver() {
        boolean value = false;
        if (hexEncoder.get() > 242 && hexEncoder.get() < 312) { //* 1.5) {
            value = true;
        }
        Logger.recordOutput("Intake/Hex Encoder", hexEncoder.get());
        return value;
    }

    public Command setCollectorPower(double speed) {
        return runOnce(() -> this.collectSparkMax.set(speed));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/Sensor Far", m_noteSwitch.get());
        SmartDashboard.putNumber("collect_velocity", this.collectEncoder.getVelocity());
        SmartDashboard.putNumber("collector_angle_absolute",
                this.angleSparkMax.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        SmartDashboard.putNumber("HexEncoderCount", this.hexEncoder.get());
    }
}
