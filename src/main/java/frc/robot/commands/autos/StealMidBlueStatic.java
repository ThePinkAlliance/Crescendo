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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.PinkPIDConstants;
import frc.lib.pathing.ChoreoUtil;
import frc.lib.pathing.events.ChoreoEvent;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeActions;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.commands.shooter.SmartLuanch;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;
import java.util.function.Consumer;

public class StealMidBlueStatic {
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
        TRANSFER_1,
        SHOOT_1,
        COLLECT_2,
        TRANSFER_2,
        SHOOT_2,
        COLLECT_3,
        TRANSFER_3,
        SHOOT_3,
        STOP,
        KILL,
    }

    enum ActionState {
        INIT,
        EXEC,
        TERM,
    }

    private static ActionSteps actionStep = ActionSteps.PREP_COLLECT_1;
    private static ActionState actionState = ActionState.INIT;

    public static Command getLeft(SwerveSubsystem swerveSubsystem, TurretSubsystem m_turret, Intake m_intake,
            Angle m_angle,
            VisionSubsystem m_visionSubsystem, Shooter m_shooter) {
        StealMidBlueStatic.s_angle = m_angle;
        StealMidBlueStatic.s_shooter = m_shooter;
        StealMidBlueStatic.s_turret = m_turret;
        StealMidBlueStatic.s_intake = m_intake;

        var path_1 = Choreo.getTrajectory("steal-mid2b.1");
        var path_2 = Choreo.getTrajectory("steal-mid2b.2");
        var path_3 = Choreo.getTrajectory("steal-mid2b.3");
        Pose2d path_pose_1 = path_1.getInitialPose();

        var built_path_1 = StealMidBlueStatic.buildAutoFollower(swerveSubsystem, path_1,
                StealMidBlueStatic::pathObserver1, .75);
        var built_path_2 = StealMidBlueStatic.buildAutoFollower(swerveSubsystem, path_2,
                StealMidBlueStatic::pathObserver2, .85);
        var built_path_3 = StealMidBlueStatic.buildAutoFollower(swerveSubsystem, path_3,
                StealMidBlueStatic::pathObserver3, .85);
        var path_sequence_1 = new SequentialCommandGroup(
                m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                new ParallelCommandGroup(
                        m_turret.setTargetPosition(112), new ShootNoteAuto(40, -3800, m_shooter, m_angle,
                                m_visionSubsystem)),
                built_path_1);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    swerveSubsystem.resetPose(new Pose2d(path_pose_1.getX(), path_pose_1.getY(),
                            path_pose_1.getRotation()));
                    starting_pose = new Pose2d(path_pose_1.getX(), path_pose_1.getY(),
                            path_pose_1.getRotation());
                }, swerveSubsystem),
                path_sequence_1,
                m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                m_turret.setTargetPosition(0).alongWith(
                        built_path_2),
                m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS),
                m_turret.setTargetPosition(
                        0).alongWith(built_path_3),
                Commands.runOnce(() -> {
                    swerveSubsystem.setStates(new ChassisSpeeds());
                    m_shooter.stop();
                    m_intake.stop();
                }, swerveSubsystem));
    }

    private static Command buildAutoFollower(SwerveSubsystem swerveSubsystem, ChoreoTrajectory path,
            Consumer<Pose2d> pathObserver) {
        return buildAutoFollower(swerveSubsystem, path, pathObserver, 0);
    }

    private static Command buildAutoFollower(SwerveSubsystem swerveSubsystem, ChoreoTrajectory path,
            Consumer<Pose2d> pathObserver, double extraTime) {
        PinkPIDConstants translation_y_constants = new PinkPIDConstants(5, 0.0, 0.0);
        PinkPIDConstants translation_x_constants = new PinkPIDConstants(5, 0.0, 0.0);
        PinkPIDConstants rotation_constants = new PinkPIDConstants(3, 0.4, 0);

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
                swerveSubsystem::setStates, pathObserver, extraTime, () -> false,
                swerveSubsystem);
    }

    private static void pathObserver1(Pose2d pose) {
        if (s_shooter == null || s_angle == null || s_turret == null) {
            return;
        }

        if (actionStep == ActionSteps.START) {
            if (actionState == ActionState.INIT) {
                timer.start();
                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                double starting_delta = Math.abs(pose.getX() - starting_pose.getX());

                if (starting_delta >= .4) {
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
                timer.reset();
                timer.stop();
                actionState = ActionState.INIT;
                actionStep = ActionSteps.SHOOT_1;
            }
        }

        if (actionStep == ActionSteps.SHOOT_1) {
            var set_turret_cmd = s_turret.setTargetPosition(145);

            if (actionState == ActionState.INIT) {
                s_shooter.setVelocity(-3000);
                timer.start();
                set_turret_cmd.initialize();
                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                if ((s_shooter.isAtLeastRpm(-3000) && set_turret_cmd.isFinished()) || timer.hasElapsed(1)) {
                    s_shooter.launch(1);
                    set_turret_cmd.end(false);
                }

                if (!s_shooter.noteFound()) {
                    actionState = ActionState.TERM;
                }

                set_turret_cmd.execute();
            }

            if (actionState == ActionState.TERM) {
                s_shooter.stop();
                s_shooter.launch(0);
                timer.reset();
                timer.stop();

                actionState = ActionState.INIT;
                actionStep = ActionSteps.COLLECT_2;
            }
        }
    }

    public static void pathObserver2(Pose2d pose) {
        if (s_shooter == null || s_angle == null || s_turret == null) {
            return;
        }

        if (actionStep == ActionSteps.COLLECT_2) {
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
                    actionStep = ActionSteps.TRANSFER_2;

                    // transfer
                    s_shooter.setVelocity(2000);
                }

                Logger.recordOutput("StealMid/NoteFound2", s_intake.noteFoundSupplier().getAsBoolean());
                Logger.recordOutput("StealMid/NoteFound3", s_intake.noteFound());
            }
        }

        if (actionStep == ActionSteps.TRANSFER_2) {
            var mid_collector_command = s_intake.setAnglePosition(19.09);

            if (actionState == ActionState.INIT) {
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
                timer.reset();
                timer.stop();
                actionState = ActionState.INIT;
                actionStep = ActionSteps.SHOOT_2;
            }
        }

        if (actionStep == ActionSteps.SHOOT_2) {
            var turret_rotate = s_turret.setTargetPosition(160);

            if (actionState == ActionState.INIT) {
                s_shooter.setVelocity(-2000);
                timer.start();
                turret_rotate.initialize();
                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                turret_rotate.execute();

                if ((s_shooter.isAtLeastRpm(-2000) && turret_rotate.isFinished()) || timer.hasElapsed(1)) {
                    s_shooter.launch(1);
                    turret_rotate.end(false);
                }

                if (!s_shooter.noteFound()) {
                    actionState = ActionState.TERM;
                }
            }

            if (actionState == ActionState.TERM) {
                s_shooter.stop();
                s_shooter.launch(0);
                timer.reset();
                timer.stop();

                actionState = ActionState.INIT;
                actionStep = ActionSteps.COLLECT_3;
            }
        }
    }

    public static void pathObserver3(Pose2d pose) {
        if (s_shooter == null || s_angle == null || s_turret == null) {
            return;
        }

        if (actionStep == ActionSteps.COLLECT_3) {
            if (actionState == ActionState.INIT) {
                s_intake.setCollectorPowerRaw(1);
                s_angle.setAngleNew(5);

                if (s_shooter.noteFound()) {
                    actionStep = ActionSteps.KILL;
                } else {
                    actionState = ActionState.EXEC;
                }
            }

            if (actionState == ActionState.EXEC) {
                if (s_shooter.noteFound()) {
                    actionStep = ActionSteps.KILL;
                }

                if (s_intake.noteFoundSupplier().getAsBoolean()) {
                    Logger.recordOutput("StealMid/NoteFound", true);
                    s_intake.setCollectorPowerRaw(0);
                    actionState = ActionState.INIT;
                    actionStep = ActionSteps.TRANSFER_3;
                }

                Logger.recordOutput("StealMid/NoteFound2", s_intake.noteFoundSupplier().getAsBoolean());
                Logger.recordOutput("StealMid/NoteFound3", s_intake.noteFound());
            }
        }

        if (actionStep == ActionSteps.TRANSFER_3) {
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
                timer.reset();
                timer.stop();
                actionState = ActionState.INIT;
                actionStep = ActionSteps.SHOOT_3;
            }
        }

        if (actionStep == ActionSteps.SHOOT_3) {
            var turret_rotate = s_turret.setTargetPosition(160);

            if (actionState == ActionState.INIT) {
                s_shooter.setVelocity(-2000);
                timer.start();
                turret_rotate.initialize();
                actionState = ActionState.EXEC;
            }

            if (actionState == ActionState.EXEC) {
                turret_rotate.execute();

                if ((s_shooter.isAtLeastRpm(-2000) && turret_rotate.isFinished()) || timer.hasElapsed(1)) {
                    s_shooter.launch(1);
                    turret_rotate.end(false);
                }

                if (!s_shooter.noteFound()) {
                    actionState = ActionState.TERM;
                }
            }

            if (actionState == ActionState.TERM) {
                s_shooter.stop();
                s_shooter.launch(0);
                timer.reset();
                timer.stop();

                actionState = ActionState.INIT;
                actionStep = ActionSteps.STOP;
            }
        }
    }
}
