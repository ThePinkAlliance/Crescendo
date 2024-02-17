// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AdjustAngle extends Command {
    Angle m_angle;

    /** Creates a new AdjustAngle. */
    public AdjustAngle(Angle angle) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_angle = angle;
        addRequirements(angle);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angle = SmartDashboard.getNumber("Angle Target", 0);

        m_angle.setAngle(angle);
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
