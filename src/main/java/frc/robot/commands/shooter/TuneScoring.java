// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class TuneScoring extends Command {
    VisionSubsystem visionSubsystem;
    Angle angleSubsystem;

    /** Creates a new TuneScoring. */
    public TuneScoring(VisionSubsystem visionSubsystem, Angle angleSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.angleSubsystem = angleSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem, angleSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.angleSubsystem.resetAngle();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = visionSubsystem.getClosestTargetDistance();
        double desiredAngle = Math.atan2(80 - 15, distance) * (180 / Math.PI);

        Logger.recordOutput("Desired Angle", desiredAngle);

        this.angleSubsystem.setAngleNew(desiredAngle);
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
