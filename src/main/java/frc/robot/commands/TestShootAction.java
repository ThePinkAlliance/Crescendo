// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private Shooter.ShooterMove m_sm;
    private Timer timer;

    /** Creates a new ShootAction. */
    public TestShootAction(Shooter.ShooterMove sm, double desiredVel, double desiredAngle, Shooter shooter, Angle angle,
            Loader loader) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_angle = angle;
        this.m_shooter = shooter;
        this.m_loader = loader;
        this.desiredAngle = desiredAngle;
        this.desiredVel = desiredVel;
        this.m_sm = sm;
        this.timer = new Timer();

        addRequirements(m_shooter, m_angle, m_loader);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // this.m_shooter.setVelocity(desiredVel);
        this.timer.reset();
        this.timer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // this.m_shooter.setVelocity(desiredVel);
        // this.m_shooter.shoot(desiredVel, m_sm);
        double overrideAngle = SmartDashboard.getNumber("Angle Target", 0);
        double overrideRpm = SmartDashboard.getNumber("RpmsShooter", 0);
        double goToAngle = 0.0;
        double goToRpm = 0.0;
        if (m_sm == Shooter.ShooterMove.SHOOT) {
            goToAngle = overrideAngle;
            goToRpm = overrideRpm;
        } else {
            goToAngle = desiredAngle;
            goToRpm = desiredVel;
        }

        this.m_angle.setAngle(goToAngle);
        // this.m_shooter.shoot(goToRpm, m_sm);

        if (this.m_sm == Shooter.ShooterMove.SHOOT) {
            if (this.m_shooter.isAtLeastRpm(goToRpm) && this.timer.hasElapsed(3)) {
                m_loader.launch(2000);
            }
        } else {
            this.m_angle.setAngle(500);
            this.m_angle.resetAngle();
            m_loader.load(1200);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setVelocity(0, 0);
        m_loader.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // boolean result = false;
        // if (m_timer.hasElapsed(4))
        // result = true;
        // return result;
        return false;
    }
}
