// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAction extends Command {

    public enum IntakeActionType {
        INTAKE,
        OUTAKE,
        IDLE
    }

    IntakeActionType m_type;
    Intake m_intake;
    double m_rpm;
    BooleanSupplier m_execute;

    /** Creates a new IntakeAction. */
    public IntakeAction(BooleanSupplier execute, Intake intake, IntakeActionType type, double rpm) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = intake;
        m_type = type;
        m_rpm = rpm;
        m_execute = execute;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_type == IntakeActionType.OUTAKE) {
            m_intake.moveIntake(m_rpm);
        } else if (m_type == IntakeActionType.INTAKE) {
            m_intake.moveIntake(-m_rpm);
        } else {
            m_intake.stopIntake();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // m_execute.getAsBoolean();
    }
}
