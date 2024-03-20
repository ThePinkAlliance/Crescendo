// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteAuto extends SequentialCommandGroup {
    Shooter shooter;

    /** Creates a new ShootNoteAuto. */
    public ShootNoteAuto(double desired_angle, double desired_rpm, Shooter shooter, Angle angle,
            VisionSubsystem visionSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        this.shooter = shooter;

        addCommands(
                shooter.rampUp2(desired_rpm).alongWith(angle.GotoAngle(
                        desired_angle)),
                new SmartLuanch(shooter),
                shooter.stopShooter(), angle.setAngleCommandNew(2));
    }

    public Command compose() {
        return this.handleInterrupt(() -> {
            shooter.stop();
            shooter.setSpeed(0);
            System.out.println("COMPOSE INTERRUPT CALLED");
        });
    }
}
