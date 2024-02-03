// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Intake;

public class AdjustIntakeAngle extends Command {
    Intake m_intake;

    /** Creates a new AdjustAngle. */
    public AdjustIntakeAngle(Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_intake = intake;
        addRequirements(this.m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angle = SmartDashboard.getNumber("aAngle Target", 0);

        m_intake.setAngle(angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
