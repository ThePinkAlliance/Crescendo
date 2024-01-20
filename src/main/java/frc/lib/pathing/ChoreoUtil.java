package frc.lib.pathing;

import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.pathing.events.ChoreoEvent;
import frc.lib.pathing.events.ChoreoEventHandler;

import java.util.Optional;
import java.util.function.*;

public class ChoreoUtil {
    public static Command choreoSwerveCommand(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            BooleanSupplier mirrorTrajectory,
            Subsystem... requirements) {
        var timer = new Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    ;
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean())));
                },
                (interrupted) -> {
                    timer.stop();
                    if (interrupted) {
                        outputChassisSpeeds.accept(new ChassisSpeeds());
                    }
                },
                () -> timer.hasElapsed(trajectory.getTotalTime() + 1),
                requirements);
    }

    public static Command choreoSwerveCommandWithEvents(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            ChoreoEventHandler handler,
            BooleanSupplier mirrorTrajectory,
            Subsystem... requirements) {
        var timer = new Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    ;
                    // Does optional slow down robot loops?
                    Optional<ChoreoEvent> event = handler.compute(poseSupplier.get());

                    if (event.isPresent()) {
                        event.get().schedule();
                    }

                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean())));
                },
                (interrupted) -> {
                    timer.stop();
                    if (interrupted) {
                        outputChassisSpeeds.accept(new ChassisSpeeds());
                    }
                },
                () -> timer.hasElapsed(trajectory.getTotalTime() + 1),
                requirements);
    }

    public static ChoreoControlFunction swerveController(
            PIDController xController, PIDController yController, PIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double rotationFF = referenceState.angularVelocity;

            double xFeedback = xController.calculate(pose.getX(), referenceState.x);
            double yFeedback = yController.calculate(pose.getY(), referenceState.y);
            double rotationFeedback = rotationController.calculate(pose.getRotation().getRadians(),
                    referenceState.heading);

            SmartDashboard.putNumber("xReference", referenceState.x);
            SmartDashboard.putNumber("yReference", referenceState.y);

            SmartDashboard.putNumber("xFeedback", pose.getX());
            SmartDashboard.putNumber("yFeedback", pose.getY());

            SmartDashboard.putNumber("rotationFeedback", rotationFeedback);
            SmartDashboard.putNumber("rotationFF", rotationFF);
            SmartDashboard.putNumber("rotation", pose.getRotation().getRadians());

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
        };
    }

}
