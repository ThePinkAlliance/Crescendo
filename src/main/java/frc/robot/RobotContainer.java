// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.JoystickMap;
import frc.lib.PinkPIDConstants;
import frc.lib.pathing.ChoreoUtil;
import frc.lib.pathing.events.ChoreoEvent;
import frc.lib.pathing.events.ChoreoEventHandler;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        public SwerveSubsystem swerveSubsystem;
        public Joystick baseJoystick;

        public ChoreoTrajectory selectedTrajectory;
        public SendableChooser<Pair<ChoreoTrajectory, ChoreoEvent[]>> chooser;

        public PinkPIDConstants translation_y_constants = new PinkPIDConstants(0, 0.0, 0.0);
        public PinkPIDConstants translation_x_constants = new PinkPIDConstants(0, 0.1, 0.0);

        public PinkPIDConstants rotation_constants = new PinkPIDConstants(.15, 0, 0);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
                baseJoystick = new Joystick(0);
                this.chooser = new SendableChooser<>();

                this.chooser.addOption("Speaker Backup",
                                new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("leave_one"), null));
                this.chooser.addOption("Speaker Backup Rotate",
                                new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("leave_three"),
                                                new ChoreoEvent[] { new ChoreoEvent(
                                                                Commands.print("======= Path Finished ======="),
                                                                new Translation2d(
                                                                                4.22,
                                                                                6.48)) }));
                this.chooser.addOption("Speaker Rotate 90",
                                new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("leave_four"), null));
                this.chooser.addOption("Shoot Speaker",
                                new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("shoot_one"), null));
                this.chooser.setDefaultOption("Speaker Align",
                                new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("leave_two"),
                                                new ChoreoEvent[] {
                                                                new ChoreoEvent(Commands.runOnce(() -> System.out
                                                                                .println("====== Hi =====")),
                                                                                new Translation2d(
                                                                                                2.22, 5.36)) }));

                SmartDashboard.putData(chooser);

                // Configure the trigger bindings
                configureBindings();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                swerveSubsystem
                                .setDefaultCommand(
                                                new JoystickDrive(swerveSubsystem,
                                                                () -> baseJoystick.getRawAxis(JoystickMap.LEFT_X_AXIS),
                                                                () -> baseJoystick.getRawAxis(JoystickMap.LEFT_Y_AXIS),
                                                                () -> baseJoystick
                                                                                .getRawAxis(JoystickMap.RIGHT_X_AXIS)));

                new JoystickButton(baseJoystick, JoystickMap.BUTTON_BACK)
                                .onTrue(Commands.runOnce(() -> swerveSubsystem.resetGyro()));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                var pair = chooser.getSelected();
                ChoreoTrajectory path = pair.getFirst();
                ChoreoEvent[] events = pair.getSecond();

                ChoreoEventHandler handler = new ChoreoEventHandler(events);

                double translation_x_kP = SmartDashboard.getNumber("translation_x_kP", translation_x_constants.kP);
                double translation_x_kI = SmartDashboard.getNumber("translation_x_kI", translation_x_constants.kI);
                double translation_x_kD = SmartDashboard.getNumber("translation_x_kD", translation_x_constants.kD);

                double translation_y_kP = SmartDashboard.getNumber("translation_y_kP", translation_y_constants.kP);
                double translation_y_kI = SmartDashboard.getNumber("translation_y_kI", translation_y_constants.kI);
                double translation_y_kD = SmartDashboard.getNumber("translation_y_kD", translation_y_constants.kD);

                double rotation_kP = SmartDashboard.getNumber("rotation_kP", rotation_constants.kP);
                double rotation_kI = SmartDashboard.getNumber("rotation_kI", rotation_constants.kI);
                double rotation_kD = SmartDashboard.getNumber("rotation_kD", rotation_constants.kD);

                Pose2d path_pose = path.getInitialPose();
                swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                path_pose.getRotation()));

                // Added events to the path follower
                Command trajectory_command = ChoreoUtil.choreoSwerveCommandWithEvents(path,
                                swerveSubsystem::getCurrentPose,
                                swerveController(
                                                new PIDController(translation_x_kP, translation_x_kI, translation_x_kD,
                                                                0.01),
                                                new PIDController(translation_y_kP, translation_y_kI, translation_y_kD,
                                                                0.01),
                                                new PIDController(rotation_kP, rotation_kI, rotation_kD, 0.01)),
                                swerveSubsystem::setStates, handler, () -> false,
                                swerveSubsystem);

                return Commands
                                .sequence(
                                                trajectory_command,
                                                Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                                                swerveSubsystem));

        }

        public static ChoreoControlFunction swerveController(
                        PIDController xController, PIDController yController, PIDController rotationController) {
                rotationController.enableContinuousInput(-Math.PI, Math.PI);
                return (pose, referenceState) -> {
                        double xFF = referenceState.velocityX;
                        double yFF = referenceState.velocityY;
                        double rotationFF = referenceState.angularVelocity * -1;

                        double xFeedback = xController.calculate(pose.getX(), referenceState.x);
                        double yFeedback = yController.calculate(pose.getY(), referenceState.y);
                        double rotationFeedback = rotationController.calculate(pose.getRotation().getRadians(),
                                        referenceState.heading) * -1;

                        SmartDashboard.putNumber("xReference", referenceState.x);
                        SmartDashboard.putNumber("yReference", referenceState.y);

                        SmartDashboard.putNumber("xFeedback", pose.getX());
                        SmartDashboard.putNumber("yFeedback", pose.getY());

                        SmartDashboard.putNumber("xError", xController.getPositionError());
                        SmartDashboard.putNumber("yError", yController.getPositionError());

                        SmartDashboard.putNumber("rotationFeedback", rotationFeedback);
                        SmartDashboard.putNumber("rotationFF", rotationFF);
                        SmartDashboard.putNumber("rotation", pose.getRotation().getRadians());

                        return ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback,
                                        pose.getRotation());
                };
        }
}
