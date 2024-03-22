// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.TurretVectoring;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.concurrent.locks.Condition;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteTargetVisible extends SequentialCommandGroup {
    double target_angle = 5;

    /** Creates a new ShootNote. */
    public ShootNoteTargetVisible(Shooter shooter, Angle angle, TurretSubsystem m_turret,
            VisionSubsystem m_visionSubsystem,
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier angleSupplier) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        var pg1 = new ParallelCommandGroup(
                new LimelightAngle(angle, angleSupplier),
                new TurretVectoring(m_turret, m_visionSubsystem, () -> swerveSubsystem.getHeading()),
                shooter.rampUp2(-4800));
        var sg1 = new SequentialCommandGroup(shooter.launchNote2(),
                angle.setAngleCommandNew(5).alongWith(shooter.stopShooter()));

        var sg0 = new SequentialCommandGroup(pg1, sg1);
        addCommands(
                new ConditionalCommand(sg0, Commands.none(), () -> m_visionSubsystem.getTargetVisible()));

    }
}
