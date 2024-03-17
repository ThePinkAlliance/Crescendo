package frc.robot.commands.autos;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.subsystems.SwerveSubsystem;

public class LeaveZone {
    public static Command leftZone(SwerveSubsystem swerveSubsystem) {
        var path = Choreo.getTrajectory("leave");
        Pose2d path_pose = path.getInitialPose();

        var built_path = RobotContainer.buildAutoFollower(swerveSubsystem, path, () -> false);

        return Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                    path_pose.getRotation()));
                        }, swerveSubsystem),
                        new WaitCommand(10),
                        built_path,
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));

    }
}