// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.JoystickMap;
import frc.lib.PinkPIDConstants;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.PickupAndLoadNote;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.intake.CollectNote;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
    public VisionSubsystem m_visionSubsystem;

    public ChoreoTrajectory selectedTrajectory;
    public SendableChooser<Command> chooser;

    private Shooter m_shooter = new Shooter();
    private Angle m_angle = new Angle();
    // private Loader m_loader = new Loader();
    private Intake m_intake = new Intake();
    private Climber m_climber = new Climber();

    // i: 0.0045
    public PinkPIDConstants translation_y_constants = new PinkPIDConstants(0.12, 0.0, 0.0);
    // i: 0.005
    public PinkPIDConstants translation_x_constants = new PinkPIDConstants(0.10, 0.0, 0.0);

    public PinkPIDConstants rotation_constants = new PinkPIDConstants(0.12, 0.001, 0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
        m_visionSubsystem = new VisionSubsystem();
        baseJoystick = new Joystick(0);
        this.chooser = new SendableChooser<>();

        // this.chooser.addOption("Speaker Backup",
        // new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("leave_one"),
        // null));
        // this.chooser.addOption("Speaker Backup Rotate",
        // new Pair<ChoreoTrajectory,
        // ChoreoEvent[]>(Choreo.getTrajectory("leave_three"),
        // new ChoreoEvent[] { new ChoreoEvent(
        // Commands.print("======= Path Finished ======="),
        // new Translation2d(
        // 4.22,
        // 6.48)) }));
        // this.chooser.addOption("Speaker Rotate 90",
        // new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("leave_four"),
        // null));
        // this.chooser.addOption("Shoot Speaker",
        // new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("shoot_one"),
        // null));
        // this.chooser.setDefaultOption("Speaker Align",
        // new Pair<ChoreoTrajectory, ChoreoEvent[]>(Choreo.getTrajectory("leave_two"),
        // new ChoreoEvent[] {
        // new ChoreoEvent(Commands.runOnce(() -> System.out
        // .println("====== Hi =====")),
        // new Translation2d(
        // 2.22, 5.36)) }));

        // this.chooser.addOption("Chained Paths",
        // link_trajectory_commands(Choreo.getTrajectory("point_1"),
        // Choreo.getTrajectory("point_2")));

        SmartDashboard.putData(chooser);

        SmartDashboard.putNumber("shooter_angle", 2);

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
                                () -> baseJoystick.getRawAxis(JoystickMap.RIGHT_X_AXIS)));

        new JoystickButton(baseJoystick, JoystickMap.BUTTON_BACK)
                .onTrue(Commands.runOnce(() -> swerveSubsystem.resetGyro()));

        // For running the intake
        // new JoystickButton(baseJoystick, 4).whileTrue(
        // Commands.runOnce(() -> m_shooter.setSpeed(.3))
        // .alongWith(Commands.runOnce(() -> m_shooter.load(1))));

        // new JoystickButton(baseJoystick,
        // JoystickMap.BUTTON_A).whileTrue(m_intake.setCollectorSpeed2(.85));
        // new JoystickButton(baseJoystick,
        // JoystickMap.BUTTON_A).whileTrue(m_angle.setAngleCommandNew(25));
        // new JoystickButton(baseJoystick,
        // JoystickMap.BUTTON_B).whileTrue(m_angle.setAngleCommandNew(0));
        new JoystickButton(baseJoystick, JoystickMap.LEFT_BUMPER).whileTrue(m_intake.stowCollector());
        new JoystickButton(baseJoystick, JoystickMap.RIGHT_BUMPER).whileTrue(m_intake.deployCollector());

        new JoystickButton(baseJoystick, JoystickMap.BUTTON_A).onTrue(new CollectNote(m_intake, m_shooter, m_angle));
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y)
                .onTrue(new ShootNote(m_shooter, m_angle, m_visionSubsystem));

        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_A)
        // .onTrue(m_angle.GotoAngle(() -> SmartDashboard.getNumber("shooter_angle",
        // 0)));
        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y)
        // .whileTrue(m_shooter.loadNoteUntilFound(.3).andThen(m_shooter.rampUp2(-4800)))
        // .onFalse(Commands.runOnce(() -> m_shooter.load(-.3)));
        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_B)
        // .whileTrue(m_shooter.rampUp(0).andThen(Commands.runOnce(() ->
        // m_shooter.load(0))));

        // new POVButton(baseJoystick, JoystickMap.POV_UP).whileTrue(new
        // SetClimber(m_climber, 79, -64));
        // new POVButton(baseJoystick, JoystickMap.POV_DOWN).whileTrue(new
        // SetClimber(m_climber, 0, 0));

        // new POVButton(baseJoystick, JoystickMap.POV_LEFT)
        // .whileTrue(m_climber.resetLeftClimber());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // var pair = chooser.getSelected();
        // ChoreoTrajectory path = pair.getFirst();
        // ChoreoEvent[] events = pair.getSecond();

        // ChoreoEventHandler handler = new ChoreoEventHandler(events);

        double translation_x_kP = SmartDashboard.getNumber("translation_x_kP", translation_x_constants.kP);
        double translation_x_kI = SmartDashboard.getNumber("translation_x_kI", translation_x_constants.kI);
        double translation_x_kD = SmartDashboard.getNumber("translation_x_kD", translation_x_constants.kD);

        double translation_y_kP = SmartDashboard.getNumber("translation_y_kP", translation_y_constants.kP);
        double translation_y_kI = SmartDashboard.getNumber("translation_y_kI", translation_y_constants.kI);
        double translation_y_kD = SmartDashboard.getNumber("translation_y_kD", translation_y_constants.kD);

        double rotation_kP = SmartDashboard.getNumber("rotation_kP", rotation_constants.kP);
        double rotation_kI = SmartDashboard.getNumber("rotation_kI", rotation_constants.kI);
        double rotation_kD = SmartDashboard.getNumber("rotation_kD", rotation_constants.kD);

        // Pose2d path_pose = path.getInitialPose();
        // swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
        // path_pose.getRotation()));

        // Added events to the path follower
        // Command trajectory_command = ChoreoUtil.choreoSwerveCommandWithEvents(path,
        // swerveSubsystem::getCurrentPose,
        // swerveController(
        // new PIDController(translation_x_kP, translation_x_kI, translation_x_kD,
        // 0.02),
        // new PIDController(translation_y_kP, translation_y_kI, translation_y_kD,
        // 0.02),
        // new PIDController(rotation_kP, rotation_kI, rotation_kD, 0.02)),
        // swerveSubsystem::setStates, handler, () -> false,
        // swerveSubsystem);

        // return Commands
        // .sequence(
        // trajectory_command,
        // Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
        // swerveSubsystem));

        // return link_trajectory_commands(Choreo.getTrajectory("point_1"),
        // Choreo.getTrajectory("point_2"));

        return Commands.none();
    }

    // public Command link_trajectory_commands(ChoreoTrajectory... trajs) {
    // SequentialCommandGroup group = new SequentialCommandGroup();

    // group.addCommands(Commands.runOnce(() -> {
    // Pose2d inital_pose = trajs[0].getInitialPose();

    // swerveSubsystem.resetPose(new Pose2d(inital_pose.getX(), inital_pose.getY(),
    // inital_pose.getRotation()));
    // }));

    // for (ChoreoTrajectory traj : trajs) {
    // Command cmd = ChoreoUtil.choreoSwerveCommand(traj,
    // swerveSubsystem::getCurrentPose,
    // swerveController(
    // new PIDController(translation_x_constants.kP,
    // translation_x_constants.kI,
    // translation_x_constants.kD,
    // 0.02),
    // new PIDController(translation_y_constants.kP,
    // translation_y_constants.kI,
    // translation_y_constants.kD,
    // 0.02),
    // new PIDController(
    // rotation_constants.kP,
    // rotation_constants.kI,
    // rotation_constants.kD, 0.02)),
    // swerveSubsystem::setStates, () -> false,
    // swerveSubsystem);

    // group.addCommands(cmd);
    // }

    // return group;
    // }

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

            SmartDashboard.putNumber("xError", xController.getPositionError());
            SmartDashboard.putNumber("yError", yController.getPositionError());

            SmartDashboard.putNumber("rotationFeedback", rotationFeedback);
            SmartDashboard.putNumber("rotationFF", rotationFF);
            SmartDashboard.putNumber("rotationSetpoint", referenceState.heading);
            SmartDashboard.putNumber("rotation", pose.getRotation().getRadians());

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback,
                    pose.getRotation());
        };
    }
}
