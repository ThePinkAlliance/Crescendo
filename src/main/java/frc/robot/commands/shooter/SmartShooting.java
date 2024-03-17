// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class SmartShooting extends Command {
    Shooter m_shooter;
    Angle m_angle;
    TurretSubsystem m_turret;
    VisionSubsystem m_vision;
    SwerveSubsystem m_swerve;
    boolean is_red;

    /** Creates a new SmartShooting. */
    public SmartShooting(Shooter shooter, Angle angle, TurretSubsystem turret, SwerveSubsystem swerve,
            VisionSubsystem vision) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.m_shooter = shooter;
        this.m_turret = turret;
        this.m_angle = angle;
        this.m_vision = vision;
        this.m_swerve = swerve;
        this.is_red = true;

        addRequirements(shooter, angle, turret, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter.setVelocity(-4800);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        int speaker_id = is_red ? 4 : 7;
        Pose3d speaker_pose = Constants.FieldConstants.layout.getTagPose(speaker_id).get();
        Pose2d speaker_pose2d = new Pose2d(speaker_pose.getX(), speaker_pose.getY(),
                speaker_pose.getRotation().toRotation2d());
        double turret_heading = m_turret.getRotation().getDegrees();
        double x_accel = this.m_swerve.getAccelX().getValueAsDouble();
        // Placeholder for now until limelight tag shooting is ready.
        Pose2d robot_pose = this.m_swerve.getCurrentPose();

        Rotation2d speaker_rotation = speaker_pose2d.getRotation();
        Rotation2d turret_rotation = m_turret.getRotation();

        Rotation2d desired_turret_angle = speaker_rotation.minus(turret_rotation);

        double speaker_distance = speaker_pose2d.getTranslation()
                .getDistance(robot_pose.getTranslation());
        double future_pose_x = robot_pose.getX() + x_accel * 9.8;
        double future_distance = Math.hypot(future_pose_x, speaker_distance);
        double future_angle = (speaker_distance / future_distance) * (180 / Math.PI);

        Logger.recordOutput("Commands/SmartShooting/speaker_distance", speaker_distance);
        Logger.recordOutput("Commands/SmartShooting/speaker_id", speaker_id);
        Logger.recordOutput("Commands/SmartShooting/desired_rotation_deg", desired_turret_angle.getDegrees());
        Logger.recordOutput("Commands/SmartShooting/speaker_rotation_deg", speaker_rotation.getDegrees());
        Logger.recordOutput("Commands/SmartShooting/turret_rotation_deg", turret_rotation.getDegrees());
        Logger.recordOutput("Commands/SmartShooting/future_pose_x", future_pose_x);
        Logger.recordOutput("Commands/SmartShooting/future_pose_distance", future_distance);
        Logger.recordOutput("Commands/SmartShooting/future_angle", future_angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
