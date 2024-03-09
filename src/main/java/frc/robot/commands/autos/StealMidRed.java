// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.PinkPIDConstants;
import frc.lib.pathing.ChoreoUtil;
import frc.lib.pathing.events.ChoreoEvent;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeActions;
import frc.robot.commands.shooter.SmartLuanch;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class StealMidRed {
    private static Shooter s_shooter;
    private static Angle s_angle;
    private static TurretSubsystem s_turret;
    private static Intake s_intake;
    private static Timer timer = new Timer();
    private static Pose2d starting_pose = new Pose2d();

    enum ActionSteps {
        START,
        PREP_COLLECT_1,
        COLLECT_1,
        TRANSFER_1
    }

    enum ActionState {
        INIT,
        EXEC,
        TERM,
    }

    private static ActionSteps actionStep = ActionSteps.START;
    private static ActionState actionState = ActionState.INIT;

    private static void pathObserver(Pose2d pose) {
        if (s_shooter == null || s_angle == null || s_turret == null) {
            return;
        }

        double pose_x = pose.getX();
        double pose_y = pose.getY();

        if (actionStep == ActionSteps.START) {
            if (actionState == ActionState.INIT) {
                timer.start();
                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                double starting_delta = Math.abs(pose.getX() - starting_pose.getX());

                if (starting_delta >= .5) {
                    s_shooter.launch(1);
                }

                if (!s_shooter.noteFound()) {
                    actionState = ActionState.TERM;
                }

                Logger.recordOutput("StealMid/distance", starting_delta);
            }

            if (actionState == ActionState.TERM) {
                s_shooter.launch(0);
                s_shooter.setSpeed(0);
                timer.stop();
                timer.reset();
                actionStep = ActionSteps.PREP_COLLECT_1;
                actionState = ActionState.INIT;
            }
        }

        if (actionStep == ActionSteps.PREP_COLLECT_1) {
            var target_pos_command = s_turret.setTargetPosition(0);

            if (actionState == ActionState.INIT) {
                target_pos_command.initialize();

                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                target_pos_command.execute();

                if (target_pos_command.isFinished()) {
                    actionState = ActionState.TERM;
                }
            }

            if (actionState == ActionState.TERM) {
                target_pos_command.end(false);
                actionState = ActionState.INIT;
                actionStep = ActionSteps.COLLECT_1;
            }
        }

        if (actionStep == ActionSteps.COLLECT_1) {
            if (actionState == ActionState.INIT) {
                s_intake.setCollectorPowerRaw(1);
                s_angle.setAngleNew(5);

                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                if (s_intake.noteFoundSupplier().getAsBoolean()) {
                    Logger.recordOutput("StealMid/NoteFound", true);
                    s_intake.setCollectorPowerRaw(0);
                    actionState = ActionState.INIT;
                    actionStep = ActionSteps.TRANSFER_1;
                }

                Logger.recordOutput("StealMid/NoteFound2", s_intake.noteFoundSupplier().getAsBoolean());
                Logger.recordOutput("StealMid/NoteFound3", s_intake.noteFound());
            }
        }

        if (actionStep == ActionSteps.TRANSFER_1) {
            var mid_collector_command = s_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_MID_AUTO_POS);

            if (actionState == ActionState.INIT) {
                s_shooter.setVelocity(1500);
                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                mid_collector_command.execute();

                if (mid_collector_command.isFinished() || s_intake.getControlError() <= 2) {
                    timer.start();
                }

                if (timer.hasElapsed(.3)) {
                    s_intake.setCollectorPowerRaw(1);
                    s_shooter.load(1);
                }

                if (s_shooter.noteFound()) {
                    actionState = ActionState.TERM;
                }
            }

            if (actionState == ActionState.TERM) {
                s_intake.setCollectorPowerRaw(0);
                s_shooter.load(0);
                s_shooter.stop();
            }
        }
    }

    public static Command getLeft(SwerveSubsystem swerveSubsystem, TurretSubsystem m_turret, Intake m_intake,
            Angle m_angle,
            VisionSubsystem m_visionSubsystem, Shooter m_shooter) {
        StealMidRed.s_angle = m_angle;
        StealMidRed.s_shooter = m_shooter;
        StealMidRed.s_turret = m_turret;
        StealMidRed.s_intake = m_intake;

        var path_1 = Choreo.getTrajectory("steal-mid.1");
        var path_2 = Choreo.getTrajectory("steal-mid.2");
        var path_3 = Choreo.getTrajectory("steal-mid.3");
        Pose2d path_pose_1 = path_1.getInitialPose();

        var built_path_1 = StealMidRed.buildAutoFollower(swerveSubsystem, path_1);
        var path_sequence_1 = new SequentialCommandGroup(
                m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                new ParallelCommandGroup(
                        m_turret.setTargetPosition(135), m_angle.setAngleCommandNew(35)),
                built_path_1);
        var prepare_shooter_1 = new ParallelCommandGroup(m_shooter.rampUp2(-4800), m_angle.setAngleCommandNew(30));
        var shoot_note_1 = new SequentialCommandGroup(m_turret.setTargetPosition(180).alongWith(prepare_shooter_1),
                m_shooter.launchNote3());

        return Commands.sequence(
                Commands.runOnce(() -> {
                    swerveSubsystem.resetPose(new Pose2d(path_pose_1.getX(), path_pose_1.getY(),
                            path_pose_1.getRotation()));
                    m_shooter.setVelocity(-3800);
                    starting_pose = new Pose2d(path_pose_1.getX(), path_pose_1.getY(),
                            path_pose_1.getRotation());
                }, swerveSubsystem),
                path_sequence_1,
                Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()), swerveSubsystem));
    }

    private static Command buildAutoFollower(SwerveSubsystem swerveSubsystem, ChoreoTrajectory path,
            ChoreoEvent... events) {
        PinkPIDConstants translation_y_constants = new PinkPIDConstants(5, 0.0, 0.0);
        PinkPIDConstants translation_x_constants = new PinkPIDConstants(5, 0.0, 0.0);
        PinkPIDConstants rotation_constants = new PinkPIDConstants(3, 0.1, 0);

        return ChoreoUtil.choreoSwerveCommandWithTriggers(path,
                swerveSubsystem::getCurrentPose,
                RobotContainer.swerveController(
                        new PIDController(translation_x_constants.kP,
                                translation_x_constants.kI,
                                translation_x_constants.kD,
                                0.02),
                        new PIDController(
                                translation_y_constants.kP,
                                translation_y_constants.kI,
                                translation_y_constants.kD,
                                0.02),
                        new PIDController(
                                rotation_constants.kP,
                                rotation_constants.kI,
                                rotation_constants.kD)),
                swerveSubsystem::setStates, StealMidRed::pathObserver, () -> false,
                swerveSubsystem);
    }
}
