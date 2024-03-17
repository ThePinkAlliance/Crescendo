// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.function.DoubleSupplier;

public class LimelightAngle extends Command {
    Angle angle;
    DoubleSupplier supplier;

    /** Creates a new LimelightAngle. */
    public LimelightAngle(Angle angle, DoubleSupplier supplier) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.angle = angle;
        this.supplier = supplier;

        addRequirements(angle);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double distance = supplier.getAsDouble();
        double target_angle = (0.0145 * 0.95 * Math.pow(distance, 2) - 2.5546 * distance + 143.1) * 1.10;
        Logger.recordOutput("targetAngle", target_angle);
        Logger.recordOutput("distance", distance);

        if (distance != 135) {
            this.angle.setAngleNew(target_angle);
        }
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
