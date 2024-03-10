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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

    public static Command choreoEventCommand(ChoreoEvent[] events, Command controller) {
        SequentialCommandGroup timed_commands = new SequentialCommandGroup();

        for (ChoreoEvent event : events) {
            timed_commands.addCommands(new WaitCommand(event.getExecTime()), event.getCommand());
        }

        return controller.alongWith(timed_commands);
    }

    public static Command choreoSwerveCommandWithTriggers(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Consumer<Pose2d> trigger_handler,
            BooleanSupplier mirrorTrajectory,
            Subsystem... requirements) {
        return choreoSwerveCommandWithTriggers(trajectory, poseSupplier, controller, outputChassisSpeeds,
                trigger_handler, 0, mirrorTrajectory, requirements);
    }

    public static Command choreoSwerveCommandWithTriggers(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Consumer<Pose2d> trigger_handler,
            double extraTime,
            BooleanSupplier mirrorTrajectory,
            Subsystem... requirements) {
        var timer = new Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    ;
                    trigger_handler.accept(poseSupplier.get());

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
                () -> timer.hasElapsed(trajectory.getTotalTime() + .5 + extraTime),
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
                    Optional<ChoreoEvent> event = handler.compute(poseSupplier.get(), timer.get());

                    // This might benefit from using a parallel command.
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
                () -> timer.hasElapsed(trajectory.getTotalTime() + .5),
                requirements);
    }
}
