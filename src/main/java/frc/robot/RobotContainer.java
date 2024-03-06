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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.shooter.ShootNote;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.commands.shooter.TuneScoring;
import frc.robot.commands.shooter.TuneShootAction;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.intake.CollectNote;
import frc.robot.commands.intake.CollectNoteAuto;
import frc.robot.commands.intake.CollectNoteV2;
import frc.robot.commands.intake.CollectTransferNote;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Climber.ClimberSide;
import frc.robot.subsystems.intake.Intake;
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
    public Joystick towerJoystick;
    public VisionSubsystem m_visionSubsystem;

    public ChoreoTrajectory selectedTrajectory;
    public SendableChooser<Command> chooser;

    private Shooter m_shooter = new Shooter();
    private Angle m_angle = new Angle();
    // private Loader m_loader = new Loader();
    private Intake m_intake = new Intake();
    private TurretSubsystem m_turret = new TurretSubsystem();

    private double start_time;

    // i: 0.0045
    public PinkPIDConstants translation_y_constants = new PinkPIDConstants(5, 0.0, 0.0);
    // i: 0.005
    public PinkPIDConstants translation_x_constants = new PinkPIDConstants(5, 0.0, 0.0);

    // kP: 2, kI: 0.01;
    public PinkPIDConstants rotation_constants = new PinkPIDConstants(3, 0.1, 0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
        m_visionSubsystem = new VisionSubsystem();
        baseJoystick = new Joystick(0);
        towerJoystick = new Joystick(1);
        this.chooser = new SendableChooser<>();

        var path = Choreo.getTrajectory("red-left-two");
        Pose2d path_pose = path.getInitialPose();

        var prepare_turret = m_turret.setTargetPosition(0).alongWith(m_angle.setAngleCommand(5));

        Command p1 = ChoreoUtil.choreoEventCommand(new ChoreoEvent[] {},
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
                                new PIDController(
                                        rotation_constants.kP,
                                        rotation_constants.kI,
                                        rotation_constants.kD)),
                        swerveSubsystem::setStates, () -> false,
                        swerveSubsystem));

        var shoot_routine = new SequentialCommandGroup(new WaitCommand(0.85),
                m_turret.setTargetPositionRaw(Constants.TurretConstants.REVERSE_SHOOTING_POS).alongWith(
                        new ShootNoteAuto(47.52, -4500, m_shooter, m_angle,
                                m_visionSubsystem)));

        var shoot_routine2 = new SequentialCommandGroup(new WaitCommand(0.85),
                m_turret.setTargetPositionRaw(
                        48.96).alongWith(
                                new ShootNoteAuto(41, -4500, m_shooter, m_angle,
                                        m_visionSubsystem)));

        var command_one = Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                    path_pose.getRotation()));
                            start_time = Timer.getFPGATimestamp();
                        }, swerveSubsystem),
                        m_intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS).alongWith(shoot_routine),
                        p1.alongWith(Commands.sequence(
                                m_intake.collectUntilFound(Constants.IntakeConstants.COLLECT_DUTY_CYCLE)
                                        .alongWith(prepare_turret),
                                m_intake.goToTransfer()
                                        .alongWith(m_shooter
                                                .loadNoteUntilFound(Constants.ShooterConstants.COLLECT_DUTY_CYCLE)),
                                m_intake.setCollectorPower(
                                        0))),
                        shoot_routine2,
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));

        var path_2 = Choreo.getTrajectory("move-1m");
        var path_2_inital_pose = path_2.getInitialPose();
        var e1 = Commands.sequence(Commands.runOnce(() -> {
            swerveSubsystem.resetPose(new Pose2d(path_2_inital_pose.getX(), path_2_inital_pose.getY(),
                    path_2_inital_pose.getRotation()));
            start_time = Timer.getFPGATimestamp();
        }, swerveSubsystem), buildAutoFollower(path_2),
                Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                        swerveSubsystem));

        var path_3 = Choreo.getTrajectory("rotate-90");
        var path_3_inital_pose = path_3.getInitialPose();
        var e2 = Commands.sequence(Commands.runOnce(() -> {
            swerveSubsystem.resetPose(new Pose2d(path_3_inital_pose.getX(), path_3_inital_pose.getY(),
                    path_3_inital_pose.getRotation()));
            start_time = Timer.getFPGATimestamp();
        }, swerveSubsystem), buildAutoFollower(path_3),
                Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                        swerveSubsystem));

        this.chooser.addOption("Test Command", command_one);
        this.chooser.addOption("move-1m", e1);
        this.chooser.addOption("rotate-90", e2);
        this.chooser.addOption("Shoot Center Close", new ShootCenterClose(m_shooter, m_angle));

        SmartDashboard.putData(chooser);
        SmartDashboard.putNumber("shooter_angle", 2);

        // Configure the trigger bindings
        configureBindings();
    }

    public Command buildAutoFollower(ChoreoTrajectory path, ChoreoEvent... events) {
        return ChoreoUtil.choreoEventCommand(events,
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
                                new PIDController(
                                        rotation_constants.kP,
                                        rotation_constants.kI,
                                        rotation_constants.kD)),
                        swerveSubsystem::setStates, () -> false,
                        swerveSubsystem));
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

        new JoystickButton(baseJoystick, JoystickMap.LEFT_BUMPER).whileTrue(m_intake.setAnglePosition(0));
        new JoystickButton(baseJoystick, JoystickMap.RIGHT_BUMPER)
                .whileTrue(new CollectNoteV2(m_intake, m_shooter, m_angle,
                        m_turret))
                .onFalse(
                        m_intake.setCollectorPower(0));

        new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y)
                .whileTrue(Commands.runOnce(() -> m_shooter.setVelocity(-500))).onFalse(
                        Commands.runOnce(() -> m_shooter.setSpeed(0)));
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_B)
                .whileTrue(Commands.runOnce(() -> m_shooter.setVelocity(-4800))).onFalse(
                        Commands.runOnce(() -> m_shooter.setSpeed(0)));
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_A)
                .whileTrue(new ShootNote(m_shooter, m_angle, m_visionSubsystem))
                .onFalse(Commands.runOnce(() -> m_shooter.setSpeed(0)));

        // Tower
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_X)
                .onTrue(m_turret.setTargetPosition(180));
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_A)
                .onTrue(m_turret.setTargetPosition(0));
        m_turret.setDefaultCommand(
                Commands.run(() -> m_turret.set(towerJoystick.getRawAxis(JoystickMap.LEFT_X_AXIS)), m_turret));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
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

            Logger.recordOutput("Auto/Rotation Error", rotationController.getPositionError());

            Logger.recordOutput("Auto/X Reference", referenceState.x);
            Logger.recordOutput("Auto/Y Reference", referenceState.y);

            Logger.recordOutput("Auto/X Feedback", xFeedback);
            Logger.recordOutput("Auto/Y Feedback", yFeedback);

            Logger.recordOutput("Auto/X Error", xController.getPositionError());
            Logger.recordOutput("Auto/Y Error", yController.getPositionError());

            Logger.recordOutput("rotationFeedback", rotationFeedback);
            Logger.recordOutput("rotationFF", rotationFF);
            Logger.recordOutput("rotationSetpoint", referenceState.heading);
            Logger.recordOutput("rotation", pose.getRotation().getRadians());

            // Reverse the sum of x so it moves forward and backwards on the field
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    (xFF + xFeedback) * -1, (yFF + yFeedback) * -1, rotationFF - rotationFeedback,
                    pose.getRotation());
        };
    }
}
