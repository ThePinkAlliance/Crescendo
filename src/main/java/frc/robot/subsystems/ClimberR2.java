// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberR2 extends SubsystemBase {
    /** Creates a new ClimberR2. */
    private static final int leftClimberID = 24;
    private static final int rightClimberID = 23;
    private double rightEncoder0;
    private double leftEncoder0;

    private TalonFX leftClimber;
    private TalonFX rightClimber;

    private double kP = 0.1;
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

        leftClimber = new TalonFX(leftClimberID, "rio");
        rightClimber = new TalonFX(rightClimberID, "base");
        leftClimber.setNeutralMode(NeutralModeValue.Brake);
        rightClimber.setNeutralMode(NeutralModeValue.Brake);

        // Innitial positions (Used for deltas)
        leftEncoder0 = leftClimber.getPosition().getValueAsDouble();
        rightEncoder0 = rightClimber.getPosition().getValueAsDouble();
    }

    public Command setTarget(double leftT, double rightT) {
        Timer w = new Timer();
        double posTolerance = 1, timeToleranceSec = 1.5;

        double leftTarget = SmartDashboard.getBoolean("Climber use Shuffleboard", false)
                ? SmartDashboard.getNumber("Left Target", 0)
                : leftT;
        double rightTarget = SmartDashboard.getBoolean("Climber use Shuffleboard", false)
                ? SmartDashboard.getNumber("Right Target", 0)
                : rightT;
        return new FunctionalCommand(
                () -> { // Innit
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
                        double leftPower = this.leftController.calculate(leftPos);
                        System.out.println("Left Motor Power: " + leftPower);
                        this.leftClimber.set(leftPower);
                    }
                    if (!this.rightArrived) {
                        double rightPower = this.rightController.calculate(rightPos);
                        System.out.println("Right Motor Power: " + rightPower + "\n");
                        this.rightClimber.set(rightPower);
                    }
                },
                (i) -> { // On End
                    this.leftClimber.set(0);
                    this.rightClimber.set(0);
                },
                () -> (/* Is it done? */ (this.leftArrived && this.rightArrived) || w.get() >= timeToleranceSec),
                this);
    }

    public void resetEncoder() {
        leftEncoder0 = this.leftClimber.getPosition().getValueAsDouble();
        rightEncoder0 = this.rightClimber.getPosition().getValueAsDouble();
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
    }
}
