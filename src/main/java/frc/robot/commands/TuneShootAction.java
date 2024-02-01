// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;

public class TuneShootAction extends Command {
    private Shooter m_shooter;
    private Angle m_angle;

    /** Creates a new ShootAction. */
    public TuneShootAction(Shooter shooter, Angle angle) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_shooter = shooter;
        this.m_angle = angle;

        addRequirements(shooter, angle);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double dashGrey = SmartDashboard.getNumber("grey Target", 0);
        double dashGreen = SmartDashboard.getNumber("green Target", 0);
        double dashTarget = SmartDashboard.getNumber("both Target", 0);

        m_shooter.setMotionMagicRps(dashGrey, dashGreen, dashTarget);

        m_shooter.putValues();
        m_shooter.setInvertedMotors();
        m_shooter.adjustPID();

        boolean isHardStopped = SmartDashboard.getBoolean("stop", false);

        if (isHardStopped) {
            SmartDashboard.putNumber("green Target", 0);
            SmartDashboard.putNumber("grey Target", 0);
            SmartDashboard.putNumber("both Target", 0);
        }

        /*
         * ANGLE CONTROL
         */
        m_angle.setAnglePID();
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
