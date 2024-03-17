// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;

/** Add your docs here. */
public class SweepNotesBlue {
    public static Command getRight(SwerveSubsystem swerveSubsystem, TurretSubsystem m_turret, Intake m_intake,
            Angle m_angle,
            VisionSubsystem m_visionSubsystem, Shooter m_shooter) {
        var path_1 = Choreo.getTrajectory("sweep-blue.1");
        var path_2 = Choreo.getTrajectory("sweep-blue.2");
        var path_3 = Choreo.getTrajectory("sweep-blue.3");
        Pose2d path_pose_1 = path_1.getInitialPose();

        var prepare_turret_1 = m_turret.setTargetPosition(0).alongWith(m_angle.setAngleCommand(5));
        var prepare_turret_2 = m_turret.setTargetPosition(0).alongWith(m_angle.setAngleCommand(5));
        var prepare_turret_3 = m_turret.setTargetPosition(0).alongWith(m_angle.setAngleCommand(5));

        var built_path_1 = RobotContainer.buildAutoFollower(swerveSubsystem, path_1);
        var built_path_2 = RobotContainer.buildAutoFollower(swerveSubsystem, path_2);
        var built_path_3 = RobotContainer.buildAutoFollower(swerveSubsystem, path_3);
        var shoot_routine_1 = new SequentialCommandGroup(
                m_turret.setTargetPositionRaw(
                        78.37)
                        .alongWith(
                                new ShootNoteAuto(45, -3900, m_shooter, m_angle,
                                        m_visionSubsystem)));
        var target_command_1 = new ParallelCommandGroup(m_turret.setTargetPositionRaw(
                -50),
                new ShootNoteAuto(36, -3900, m_shooter, m_angle,
                        m_visionSubsystem));

        var target_command_2 = new ParallelCommandGroup(m_turret.setTargetPositionRaw(
                62.71),
                new ShootNoteAuto(36, -3800, m_shooter, m_angle,
                        m_visionSubsystem));

        // var target_command_3 = new
        // ParallelCommandGroup(m_turret.setTargetPositionRaw(
        // -30),
        // new ShootNoteAuto(41, -3800, m_shooter, m_angle,
        // m_visionSubsystem));

        return Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose_1.getX(), path_pose_1.getY(),
                                    path_pose_1.getRotation()));
                        }, swerveSubsystem),
                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                        shoot_routine_1,
                        built_path_1.alongWith(Commands.sequence(
                                m_intake.collectUntilFound(Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE)
                                        .alongWith(prepare_turret_1),
                                new SequentialCommandGroup(
                                        m_intake.setAnglePosition(
                                                19.95),
                                        m_intake.setCollectorPower(
                                                Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE))
                                        .alongWith(
                                                m_shooter
                                                        .loadNoteUntilFound2(
                                                                2000)),
                                m_intake.setCollectorPower(
                                        0))),
                        target_command_1,
                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                        built_path_2.alongWith(Commands.sequence(
                                m_intake.collectUntilFound(Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE)
                                        .alongWith(prepare_turret_2),
                                new SequentialCommandGroup(
                                        m_intake.setAnglePosition(
                                                19.95),
                                        m_intake.setCollectorPower(
                                                Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE))
                                        .alongWith(
                                                m_shooter
                                                        .loadNoteUntilFound2(
                                                                2000)),
                                m_intake.setCollectorPower(
                                        0))),
                        target_command_2,
                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                        built_path_3.alongWith(Commands.sequence(
                                m_intake.collectUntilFound(Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE)
                                        .alongWith(prepare_turret_3),
                                new SequentialCommandGroup(
                                        m_intake.setAnglePosition(
                                                19.95),
                                        m_intake.setCollectorPower(
                                                Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE))
                                        .alongWith(
                                                m_shooter
                                                        .loadNoteUntilFound2(
                                                                2000)),
                                m_intake.setCollectorPower(
                                        0))),
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));
    }
}
