// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShot extends SequentialCommandGroup {
    /** Creates a new AmpShot. */
    public AmpShot(Intake intake, SwerveSubsystem swerveSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(intake.setAnglePosition(0).alongWith(
                driveForSeconds(.15, swerveSubsystem)),
                new WaitCommand(.15),
                intake.setCollectorPower(-1).alongWith(new WaitCommand(
                        .5)),
                intake.setCollectorPower(0));
    }

    public Command driveForSeconds(double time, SwerveSubsystem swerveSubsystem) {
        Timer timer = new Timer();

        return new FunctionalCommand(() -> {
            timer.reset();
            timer.start();
        }, () -> {
            swerveSubsystem.setSpeedModules(-1);
        }, (i) -> {
            swerveSubsystem.setSpeedModules(0);
            timer.stop();
        }, () -> timer.hasElapsed(time), swerveSubsystem);
    }

    public ChassisSpeeds calculatePower(double x, double y, SwerveSubsystem swerveSubsystem) {
        Rotation2d robotAngle = swerveSubsystem.getRotation2d();
        double xField = x * robotAngle.getSin() + y * robotAngle.getCos();
        double yField = x * robotAngle.getCos() + y * -robotAngle.getSin();

        Logger.recordOutput("AmpShot/xField", xField);
        Logger.recordOutput("AmpShot/yField", yField);
        Logger.recordOutput("AmpShot/Rotation", robotAngle.getDegrees());
        Logger.recordOutput("AmpShot/Heading", swerveSubsystem.getHeading());
        Logger.recordOutput("AmpShot/Yaw", swerveSubsystem.getYaw());

        return new ChassisSpeeds(xField, yField, 0);
    }
}
