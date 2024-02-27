// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.JoystickMap;
import frc.lib.PinkPIDConstants;
import frc.lib.pathing.ChoreoUtil;
import frc.lib.pathing.events.ChoreoEvent;
import frc.lib.pathing.events.ChoreoEventHandler;
import frc.robot.commands.AdjustIntakeAngle;
import frc.robot.commands.PickupAndLoadNote;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.SetClimber;
import frc.robot.commands.autos.ShootCenterClose;
import frc.robot.commands.shooter.AdjustAngle;
import frc.robot.commands.shooter.AlignShoot;
import frc.robot.commands.shooter.ShootAction;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.commands.shooter.TuneScoring;
import frc.robot.commands.shooter.TuneShootAction;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.intake.CollectNote;
import frc.robot.commands.intake.CollectNoteAuto;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Climber.ClimberSide;
import org.littletonrobotics.junction.Logger;

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
    public PinkPIDConstants translation_y_constants = new PinkPIDConstants(5, 0.0, 0.0);
    // i: 0.005
    public PinkPIDConstants translation_x_constants = new PinkPIDConstants(5, 0.0, 0.0);

    public PinkPIDConstants rotation_constants = new PinkPIDConstants(3.5, 0.0, 0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
        m_visionSubsystem = new VisionSubsystem();
        baseJoystick = new Joystick(0);
        this.chooser = new SendableChooser<>();

        var path = Choreo.getTrajectory("shoot-two.1");
        var path2 = Choreo.getTrajectory("shoot-two.2");

        Pose2d path_pose = path.getInitialPose();

        // Added events to the path follower
        Command p1 = ChoreoUtil.choreoEventCommand(new ChoreoEvent[] {
                new ChoreoEvent(new CollectNoteAuto(m_intake, m_shooter, m_angle),
                        0.32) },
                ChoreoUtil.choreoSwerveCommand(path,
                        swerveSubsystem::getCurrentPose,
                        swerveController(
                                new PIDController(translation_x_constants.kP,
                                        translation_x_constants.kI,
                                        translation_x_constants.kD,
                                        0.02),
                                new PIDController(
                                        translation_y_constants.kP,
                                        translation_y_constants.kI,
                                        translation_y_constants.kD,
                                        0.02),
                                new ProfiledPIDController(
                                        rotation_constants.kP,
                                        rotation_constants.kI,
                                        rotation_constants.kD, new Constraints(10, 7))),
                        swerveSubsystem::setStates, () -> false,
                        swerveSubsystem));

        Command p2 = ChoreoUtil.choreoEventCommand(new ChoreoEvent[] {},
                ChoreoUtil.choreoSwerveCommand(path2,
                        swerveSubsystem::getCurrentPose,
                        swerveController(
                                new PIDController(translation_x_constants.kP,
                                        translation_x_constants.kI,
                                        translation_x_constants.kD,
                                        0.02),
                                new PIDController(
                                        translation_y_constants.kP,
                                        translation_y_constants.kI,
                                        translation_y_constants.kD,
                                        0.02),
                                new ProfiledPIDController(
                                        rotation_constants.kP,
                                        rotation_constants.kI,
                                        rotation_constants.kD, new Constraints(10, 7))),
                        swerveSubsystem::setStates, () -> false,
                        swerveSubsystem));

        var command_one = Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                    path_pose.getRotation()));
                        }, swerveSubsystem),
                        new ShootNoteAuto(54, -4200, m_shooter, m_angle, m_visionSubsystem),
                        Commands.waitSeconds(0.5),
                        p1,
                        p2,
                        new AlignShoot(20, swerveSubsystem, m_visionSubsystem).alongWith(
                                new ShootNoteAuto(42, -4800, m_shooter, m_angle,
                                        m_visionSubsystem)),
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));

        this.chooser.addOption("Choreo Path 1", command_one);
        this.chooser.addOption("Shoot Center Close", new ShootCenterClose(m_shooter, m_angle));

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
        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y)
        // .onTrue(new PickupAndLoadNote(m_intake, m_shooter, m_angle,
        // m_visionSubsystem));

        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_A)
        // .onTrue(new FunctionalCommand(() -> {
        // }, () -> {
        // var angle = SmartDashboard.getNumber("shooter_angle", 0);
        // m_angle.setAngleNew(angle);
        // }, (e) -> {
        // m_angle.stop();
        // }, () -> true, m_angle));
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_A).onTrue(new CollectNote(m_intake, m_shooter, m_angle));

        double f = 1;

        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_X)
        // .onTrue(m_shooter.rampUp2(-600 * f,
        // -3000 * f))
        // .onFalse(m_shooter.launchNote2().andThen(m_shooter.stopShooter()));
        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_B)
        // .onTrue(m_shooter.loadNoteUntilFound(.5).andThen(m_shooter.rampUp2(
        // 200)))
        // .onFalse(m_shooter.stopShooter());

        new JoystickButton(baseJoystick, JoystickMap.BUTTON_X)
                .whileTrue(m_intake.setCollectorPower(1)).onFalse(m_intake.setCollectorPower(0));
        new JoystickButton(baseJoystick,
                JoystickMap.BUTTON_B).whileTrue(m_intake.setCollectorPower(-1))
                .onFalse(m_intake.setCollectorPower(0));

        new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y)
                .onTrue(new ShootNote(m_shooter, m_angle, m_visionSubsystem));

        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_X)
        // .whileTrue(m_shooter.loadNoteUntilFound(.3).andThen(m_shooter.rampUp2(-4800)))
        // .onFalse(Commands.runOnce(() -> m_shooter.load(-.3)));
        // new JoystickButton(baseJoystick, JoystickMap.BUTTON_B)
        // .whileTrue(m_shooter.rampUp(-400).andThen(Commands.runOnce(() ->
        // m_shooter.load(-.3))))
        // .onFalse(Commands.runOnce(() -> m_shooter.load(0)));

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
        // ChoreoTrajectory path = chooser.getSelected();
        // ChoreoEvent[] events = pair.getSecond();

        // ChoreoEventHandler handler = new ChoreoEventHandler(events);

        return chooser.getSelected();

        // return link_trajectory_commands(Choreo.getTrajectory("point_1"),
        // Choreo.getTrajectory("point_2"));

        // return Commands.none();
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
            PIDController xController, PIDController yController, ProfiledPIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double rotationFF = referenceState.angularVelocity;

            double xFeedback = xController.calculate(pose.getX(), referenceState.x);
            double yFeedback = yController.calculate(pose.getY(), referenceState.y);
            double rotationFeedback = rotationController.calculate(pose.getRotation().getRadians(),
                    referenceState.heading);

            Logger.recordOutput("Auto/X Reference", referenceState.x);
            Logger.recordOutput("Auto/Y Reference", referenceState.y);

            Logger.recordOutput("Auto/X Feedback", xFeedback);
            Logger.recordOutput("Auto/Y Feedback", yFeedback);

            Logger.recordOutput("Auto/X Error", xController.getPositionError());
            Logger.recordOutput("Auto/Y Error", yController.getPositionError());

            Logger.recordOutput("rotationFeedback", rotationFeedback);
            Logger.recordOutput("rotationFF", rotationFF);
            Logger.recordOutput("rotationSetpoint", referenceState.heading);
            Logger.recordOutput("rotation", Math.abs(pose.getRotation().getRadians()));

            // Reverse the sum of x so it moves forward and backwards on the field
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    (xFF + xFeedback) * -1, (yFF + yFeedback) * -1, (rotationFF + rotationFeedback) * -1,
                    pose.getRotation());
        };
    }
}
