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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.Intake;
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
    private Climber m_climber = new Climber();
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

        var path1 = Choreo.getTrajectory("three.1");
        var path2 = Choreo.getTrajectory("three.2");
        var path3 = Choreo.getTrajectory("three.3");
        Pose2d path_pose = path1.getInitialPose();
        var prepare_shooter = new ParallelCommandGroup(m_angle.setAngleCommand(4), m_turret.setTargetPosition(
                0));

        Command p1 = buildAutoFollower(path1, new ChoreoEvent(m_intake.setAnglePosition(730, 0.85).andThen(
                prepare_shooter.alongWith(m_intake.collectUntilFound(.85))), 0.0));
        Command p2 = buildAutoFollower(path2);

        Command transfer = new SequentialCommandGroup(
                m_intake.goToTransfer()
                        .alongWith(m_shooter.loadNoteUntilFound(0.35)));

        Command shoot = new ShootNoteAuto(38, -4200, m_shooter, m_angle, m_visionSubsystem)
                .alongWith(m_turret.setTargetPosition(96));

        var command_one = Commands
                .sequence(
                        Commands.runOnce(() -> {
                            swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(),
                                    path_pose.getRotation()));
                            start_time = Timer.getFPGATimestamp();
                        }, swerveSubsystem),
                        p1,
                        transfer.andThen(shoot),
                        m_intake.setCollectorPower(0),
                        Commands.runOnce(() -> swerveSubsystem.setStates(new ChassisSpeeds()),
                                swerveSubsystem));

        this.chooser.addOption("Choreo Path 1", command_one);
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

        new JoystickButton(baseJoystick, JoystickMap.LEFT_BUMPER).whileTrue(m_intake.stowCollector());
        new POVButton(baseJoystick, 270).onTrue(m_intake.ampShot());
        new POVButton(baseJoystick, 180).whileTrue(m_intake.deployCollector(.25));
        new POVButton(baseJoystick, 0).whileTrue(m_intake.stowCollector());
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y).whileTrue(m_intake.collectUntilFound(1));

        new JoystickButton(baseJoystick, JoystickMap.RIGHT_BUMPER)
                .whileTrue(new ConditionalCommand(new CollectTransferNote(m_intake, m_shooter, m_angle, m_turret),
                        new CollectNoteV2(m_intake, m_shooter, m_angle,
                                m_turret),
                        () -> m_intake.noteFound()))
                .onFalse(
                        m_intake.setCollectorPower(0));
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_X).whileTrue(m_intake.runIntake());
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_B).onTrue(m_turret.setTargetPosition(180));
        //new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y).onTrue(m_turret.setTargetPosition(0));
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
            Logger.recordOutput("rotation", Math.abs(pose.getRotation().getRadians()));

            // Reverse the sum of x so it moves forward and backwards on the field
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    (xFF + xFeedback) * -1, (yFF + yFeedback) * -1, (rotationFF - rotationFeedback),
                    pose.getRotation());
        };
    }
}
