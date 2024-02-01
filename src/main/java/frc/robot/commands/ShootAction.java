// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;

public class ShootAction extends Command {
    private Shooter m_shooter;
    private Angle m_angle;
    private double desiredVel;
    private double desiredAngle;

    /** Creates a new ShootAction. */
    public ShootAction(double desiredVel, double desiredAngle, Shooter shooter, Angle angle) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_angle = angle;
        this.m_shooter = shooter;

        addRequirements(shooter, angle);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.m_shooter.setVelocity(desiredVel);
        this.m_angle.setAngle(desiredAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
