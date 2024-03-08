// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;

public class ShooterTune extends Command {
    Shooter shooter;
    Angle angle;
    boolean isFinished;
    Timer timer;

    /** Creates a new ShooterTune. */
    public ShooterTune(Shooter shooter, Angle angle) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.angle = angle;
        this.shooter = shooter;
        this.isFinished = false;
        this.timer = new Timer();

        SmartDashboard.putNumber("angle", 25);

        addRequirements(shooter, angle);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
        timer.reset();
        timer.stop();

        double target_angle = SmartDashboard.getNumber("angle", 25);

        shooter.setVelocity(-4800);
        angle.setAngleNew(target_angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shooter.isAtLeastRpm(-4800) && angle.getControlError() <= 1) {
            shooter.load(-1);
            timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.shooter.setSpeed(0);
        this.shooter.load(0);
        this.timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}
