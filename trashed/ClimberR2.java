// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberR2 extends SubsystemBase {
    /** Creates a new ClimberR2. */
    private static final int leftClimberID = 24;
    private static final int rightClimberID = 23;
    private double rightEncoder0;
    private double leftEncoder0;

    private TalonFX leftClimber;
    private TalonFX rightClimber;

    private double kP = 0.3;
    private double kI = 0.0;
    private double kD = 0.0;
    private PIDController leftController = new PIDController(kP, kI, kD);
    private PIDController rightController = new PIDController(kP, kI, kD);

    private double leftPos = 0.0, rightPos = 0.0;
    private boolean leftArrived = false, rightArrived = false;

    public ClimberR2() {
        SmartDashboard.putBoolean("Climber use Shuffleboard", false);
        SmartDashboard.putNumber("Left Target", 0);
        SmartDashboard.putNumber("Right Target", 0);

        leftClimber = new TalonFX(leftClimberID, "base");
        rightClimber = new TalonFX(rightClimberID, "rio");
        leftClimber.setInverted(true);

        var rightConfigs = new Slot0Configs();
        var leftConfigs = new Slot0Configs();

        rightConfigs.kP = 0.5;
        leftConfigs.kP = 0.5;

        leftClimber.getConfigurator().apply(leftConfigs);
        rightClimber.getConfigurator().apply(rightConfigs);
        leftClimber.setNeutralMode(NeutralModeValue.Brake);
        rightClimber.setNeutralMode(NeutralModeValue.Brake);

        // Innitial positions (Used for deltas)
        leftEncoder0 = leftClimber.getPosition().getValueAsDouble();
        rightEncoder0 = rightClimber.getPosition().getValueAsDouble();
        resetEncoders();
    }

    public void setClimberPos(double left, double right) {
        var right_control = new PositionVoltage(right);
        var left_control = new PositionVoltage(left);

        this.leftClimber.setControl(left_control);
        this.rightClimber.setControl(right_control);
    }

    public Command travelToClimberPos(double left, double right) {
        Timer timer = new Timer();
        return new FunctionalCommand(() -> {
            timer.reset();
            timer.start();
            this.setClimberPos(left, right);
        }, () -> {
        }, (i) -> {
        }, () -> (Math.abs((left - this.getLeftPosition())) <= 2 && Math
                .abs((right - this.getRightPosition())) <= 2) || timer.hasElapsed(4), this);
    }

    public Command setTarget(double leftT, double rightT) {
        Timer w = new Timer();
        double posTolerance = 1; // NO LONGER used in isFinished() to allow power to
        // be applied longer otherwise we do not stay in a climb position after end of
        // game
        double timeToleranceSec = 12;// Changed to 12 secs from 1.5 to keep applying power (thus holding)
        // Climb needs to start no earlier than 12 secs and no later than 5 secs before
        // end of time.

        double leftTarget = SmartDashboard.getBoolean("Climber use Shuffleboard", false)
                ? SmartDashboard.getNumber("Left Target", 0)
                : leftT;
        double rightTarget = SmartDashboard.getBoolean("Climber use Shuffleboard", false)
                ? SmartDashboard.getNumber("Right Target", 0)
                : rightT;
        return new FunctionalCommand(
                () -> { // Init
                    w.start();
                    this.leftController.setSetpoint(leftTarget);
                    this.rightController.setSetpoint(rightTarget);
                },
                () -> { // Execute
                    this.leftPos = this.leftClimber.getPosition().getValueAsDouble() - this.leftEncoder0;
                    this.rightPos = this.rightClimber.getPosition().getValueAsDouble() - this.rightEncoder0;
                    this.leftArrived = Math.abs(leftPos - leftTarget) <= posTolerance;
                    this.rightArrived = Math.abs(rightPos - rightTarget) <= posTolerance;
                    // Handle different directions
                    if (!this.leftArrived) {
                        double leftPower = this.leftController.calculate(leftPos) * 0.02;
                        System.out.println("Left Motor Power: " + leftPower);
                        this.leftClimber.set(leftPower);
                    } else {
                        this.leftClimber.set(0);
                    }
                    if (!this.rightArrived) {
                        double rightPower = this.rightController.calculate(rightPos) * 0.02;
                        System.out.println("Right Motor Power: " + rightPower + "\n");
                        this.rightClimber.set(rightPower);
                    } else {
                        this.rightClimber.set(0);
                    }
                },
                (i) -> { // On End
                    this.leftClimber.set(0);
                    this.rightClimber.set(0);
                    w.stop();
                    w.reset();
                },
                // Commented out the use of leftArrived and rightArrived - see comment above
                () -> (/* Is it done? */ /* (this.leftArrived && this.rightArrived) || */ w.get() >= timeToleranceSec),
                this);
    }

    public double getRightPosition() {
        return this.rightClimber.getRotorPosition().getValueAsDouble();
    }

    public double getLeftPosition() {
        return this.leftClimber.getRotorPosition().getValueAsDouble();
    }

    public void resetEncoder() {
        leftEncoder0 = this.leftClimber.getPosition().getValueAsDouble();
        rightEncoder0 = this.rightClimber.getPosition().getValueAsDouble();
    }

    public void resetEncoders() {
        this.rightClimber.setPosition(0);
        this.leftClimber.setPosition(0);
    }

    public void stop() {
        leftClimber.stopMotor();
        rightClimber.stopMotor();
    }

    public void testPower(double testSpeedR, double testSpeedL) {
        leftClimber.set(testSpeedL);
        rightClimber.set(testSpeedR);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Right position", this.rightPos);
        SmartDashboard.putNumber("Left position", this.leftPos);

        Logger.recordOutput("Climber/Right Position", this.rightClimber.getRotorPosition().getValueAsDouble());
        Logger.recordOutput("Climber/Left Position", this.leftClimber.getRotorPosition().getValueAsDouble());
    }
}
