// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SmartShootingHorizontal extends Command {
    Shooter m_shooter;
    Angle m_angle;
    TurretSubsystem m_turret;
    VisionSubsystem m_vision;
    SwerveSubsystem m_swerve;

    /** Creates a new SmartShootingHorizontal. */
    public SmartShootingHorizontal(Shooter shooter, Angle angle, TurretSubsystem turret, SwerveSubsystem swerve,
            VisionSubsystem vision) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_shooter = shooter;
        this.m_turret = turret;
        this.m_angle = angle;
        this.m_vision = vision;
        this.m_swerve = swerve;

        addRequirements(shooter, angle, turret, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

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
        return false;
    }
}
