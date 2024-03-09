// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
    double target_angle = 5;

    /** Creates a new ShootNote. */
    public ShootNote(Shooter shooter, Angle angle, VisionSubsystem visionSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        var g = new ParallelCommandGroup(shooter.rampUp2(-4800), new LimelightAngle(angle, visionSubsystem));

        addCommands(
                new ConditionalCommand(angle.setAngleCommandNew(20), Commands.none(),
                        () -> angle.getCancoderAngle() >= 20),
                g,
                shooter.launchNote2(),
                angle.setAngleCommandNew(5).alongWith(shooter.stopShooter()));
    }
}
