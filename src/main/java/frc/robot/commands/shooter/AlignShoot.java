// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignShoot extends Command {
    VisionSubsystem visionSubsystem;
    SwerveSubsystem swerveSubsystem;
    Timer watchdog;
    Timer found_timer;
    PIDController controller;

    /** Creates a new AlignShoot. */
    public AlignShoot(double desired_heading, SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.watchdog = new Timer();
        this.controller = new PIDController(.1, 0, 0.0);
        this.controller.enableContinuousInput(0, 360);
        this.controller.setSetpoint(desired_heading);

        addRequirements(swerveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        this.watchdog.start();
    }

    @Override
    public void execute() {
        double heading = swerveSubsystem.getHeading();
        double effort = controller.calculate(heading);

        Logger.recordOutput("Align/Heading.", heading);
        Logger.recordOutput("Align/Heading Setpoint", controller.getSetpoint());

        swerveSubsystem.setStates(new ChassisSpeeds(0, 0, effort));
    }

    @Override
    public boolean isFinished() {
        return (controller.atSetpoint()) || watchdog.hasElapsed(4);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setStates(new ChassisSpeeds(0, 0, 0));

        this.watchdog.stop();
    }
}
