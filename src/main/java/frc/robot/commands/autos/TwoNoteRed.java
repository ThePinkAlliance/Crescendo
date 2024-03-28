// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.pathing.ChoreoUtil;
import frc.lib.pathing.events.ChoreoEvent;
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
public class TwoNoteRed {
    public static Command getLeft(SwerveSubsystem swerveSubsystem, TurretSubsystem m_turret, Intake m_intake,
            Angle m_angle,
            VisionSubsystem m_visionSubsystem, Shooter m_shooter) {
        var path = Choreo.getTrajectory("red-left-two");
        Pose2d path_pose = path.getInitialPose();

        var prepare_turret = m_turret.setTargetPosition(0).alongWith(m_angle.setAngleCommand(5));

        var built_path = RobotContainer.buildAutoFollower(swerveSubsystem, path);

        var shoot_routine = new SequentialCommandGroup(
                m_turret.setTargetPositionRaw(Constants.TurretConstants.REVERSE_SHOOTING_POS).alongWith(
                        new ShootNoteAuto(41, -4000, m_shooter, m_angle,
                                m_visionSubsystem)));

        var shoot_routine2 = new SequentialCommandGroup(
                m_turret.setTargetPositionRaw(
                        52.46).alongWith(
                                new ShootNoteAuto(35.5, -4000, m_shooter, m_angle,
                                        m_visionSubsystem)));

        return Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                    path_pose.getRotation()));
                        }, swerveSubsystem),
                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                        shoot_routine,
                        built_path.alongWith(Commands.sequence(
                                m_intake.collectUntilFound(Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE)
                                        .alongWith(prepare_turret),
                                new SequentialCommandGroup(
                                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_MID_AUTO_POS),
                                        m_intake.setCollectorPower(
                                                Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE))
                                        .alongWith(
                                                m_shooter
                                                        .loadNoteUntilFound2(
                                                                2000)),
                                m_intake.setCollectorPower(
                                        0))),
                        shoot_routine2,
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));
    }

    public static Command getRight(SwerveSubsystem swerveSubsystem, TurretSubsystem m_turret, Intake m_intake,
            Angle m_angle,
            VisionSubsystem m_visionSubsystem, Shooter m_shooter) {
        var path = Choreo.getTrajectory("red-left-two");
        Pose2d path_pose = path.getInitialPose();

        var prepare_turret = m_turret.setTargetPosition(0).alongWith(m_angle.setAngleCommand(5));

        var built_path = RobotContainer.buildAutoFollower(swerveSubsystem, path);

        var shoot_routine = new SequentialCommandGroup(
                m_turret.setTargetPositionRaw(
                        80.66).alongWith(
                                new ShootNoteAuto(41, -4500, m_shooter, m_angle,
                                        m_visionSubsystem)));

        var shoot_routine2 = new SequentialCommandGroup(
                m_turret.setTargetPositionRaw(
                        72.96).alongWith(
                                new ShootNoteAuto(35.5, -4500, m_shooter, m_angle,
                                        m_visionSubsystem)));

        return Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                    path_pose.getRotation()));
                        }, swerveSubsystem),
                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                        shoot_routine,
                        built_path.alongWith(Commands.sequence(
                                m_intake.collectUntilFound(Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE)
                                        .alongWith(prepare_turret),
                                new SequentialCommandGroup(
                                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_MID_AUTO_POS),
                                        m_intake.setCollectorPower(
                                                Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE))
                                        .alongWith(
                                                m_shooter
                                                        .loadNoteUntilFound2(
                                                                2000)),
                                m_intake.setCollectorPower(
                                        0))),
                        shoot_routine2,
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));
    }

    public static Command getCenter(SwerveSubsystem swerveSubsystem, TurretSubsystem m_turret, Intake m_intake,
            Angle m_angle,
            VisionSubsystem m_visionSubsystem, Shooter m_shooter) {
        var path = Choreo.getTrajectory("red-left-two");
        Pose2d path_pose = path.getInitialPose();

        var prepare_turret = m_turret.setTargetPosition(0).alongWith(m_angle.setAngleCommand(5));

        var built_path = RobotContainer.buildAutoFollower(swerveSubsystem, path);

        var shoot_routine = new SequentialCommandGroup(
                m_turret.setTargetPositionRaw(Constants.TurretConstants.REVERSE_STARTING_POS).alongWith(
                        new ShootNoteAuto(49, -4500, m_shooter, m_angle,
                                m_visionSubsystem)));

        var shoot_routine2 = new SequentialCommandGroup(
                m_turret.setTargetPositionRaw(
                        Constants.TurretConstants.REVERSE_STARTING_POS).alongWith(
                                new ShootNoteAuto(39, -4500, m_shooter, m_angle,
                                        m_visionSubsystem)));

        return Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                    path_pose.getRotation()));
                        }, swerveSubsystem),
                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS).alongWith(shoot_routine),
                        built_path.alongWith(Commands.sequence(
                                m_intake.collectUntilFound(Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE)
                                        .alongWith(prepare_turret),
                                new SequentialCommandGroup(
                                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_MID_AUTO_POS),
                                        m_intake.setCollectorPower(
                                                Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE))
                                        .alongWith(
                                                m_shooter
                                                        .loadNoteUntilFound2(
                                                                2000)),
                                m_intake.setCollectorPower(
                                        0))),
                        shoot_routine2,
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));
    }
}
