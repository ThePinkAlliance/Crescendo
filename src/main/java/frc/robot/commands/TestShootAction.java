// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class TestShootAction extends Command {
    private Shooter m_shooter;
    private Angle m_angle;
    private Loader m_loader;
    private double desiredVel;
    private double desiredAngle;
    private Timer m_timer;

    /** Creates a new ShootAction. */
    public TestShootAction(double desiredVel, double desiredAngle, Shooter shooter, Angle angle, Loader loader) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_angle = angle;
        this.m_shooter = shooter;
        this.m_loader = loader;
        this.m_timer = new Timer();

        addRequirements(m_shooter, m_angle, m_loader);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.m_shooter.setVelocity(desiredVel);
        this.m_angle.setAngle(desiredAngle);
        this.m_timer.reset();
        this.m_timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.m_shooter.isAtLeastRpm(desiredVel)) {
            if (desiredVel > 0)
                m_loader.launch(1000);
            else
                m_loader.load(1000);
                
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setVelocity(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean result = false;
        if (m_timer.hasElapsed(4))
            result = true;
        return result;
    }
}
