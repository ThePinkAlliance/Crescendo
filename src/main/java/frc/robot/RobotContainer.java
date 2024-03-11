// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.autos.StealMidRedMoving;
import frc.robot.commands.autos.StealMidRedStatic;
import frc.robot.commands.autos.SweepNotesBlue;
import frc.robot.commands.autos.SweepNotesMiniRed;
import frc.robot.commands.autos.SweepNotesRed;
import frc.robot.commands.autos.TwoNoteBlue;
import frc.robot.commands.autos.TwoNoteRed;
import frc.robot.commands.shooter.AlignShoot;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.commands.shooter.ShooterTune;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.ClimberR1;
import frc.robot.subsystems.ClimberR2;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.intake.AmpShot;
import frc.robot.commands.intake.CollectNote;
import frc.robot.commands.intake.CollectNoteAuto;
import frc.robot.commands.intake.CollectNoteV2;
import frc.robot.commands.intake.CollectTransferNote;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimberR1.ClimberSide;
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

    private Shooter m_shooter = new Shooter();
    private Angle m_angle = new Angle();
    // private Loader m_loader = new Loader();
    private Intake m_intake = new Intake();
    private TurretSubsystem m_turret = new TurretSubsystem();
    private ClimberR2 climber_r2 = new ClimberR2();
    private SendableChooser<Command> chooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
        m_visionSubsystem = new VisionSubsystem();
        baseJoystick = new Joystick(0);
        towerJoystick = new Joystick(1);
        this.chooser = new SendableChooser<>();

        this.chooser.addOption("Red Two Note Left",
                TwoNoteRed.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Red Two Note Center",
                TwoNoteRed.getCenter(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Red Two Note Right",
                TwoNoteRed.getRight(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));

        this.chooser.addOption("Blue Two Note Left",
                TwoNoteBlue.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Blue Two Note Center",
                TwoNoteBlue.getCenter(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Blue Two Note Right",
                TwoNoteBlue.getRight(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));

        this.chooser.addOption("Red Sweep Left",
                SweepNotesRed.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Blue Sweep Left",
                SweepNotesBlue.getRight(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));

        this.chooser.addOption("Red Sweep Mini Left",
                SweepNotesMiniRed.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Red Steal Mid Shoot Static",
                StealMidRedStatic.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Red Steal Mid Shoot Moving",
                StealMidRedMoving.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));

        SmartDashboard.putData(chooser);

        // Configure the trigger bindings
        configureBindings();
    }

    public static Command buildAutoFollower(SwerveSubsystem swerveSubsystem, ChoreoTrajectory path,
            ChoreoEvent... events) {
        return buildAutoFollower(swerveSubsystem, path, () -> false, events);
    }

    public static Command buildAutoFollower(SwerveSubsystem swerveSubsystem, ChoreoTrajectory path,
            BooleanSupplier mirror,
            ChoreoEvent... events) {
        PinkPIDConstants translation_y_constants = new PinkPIDConstants(5, 0.0, 0.0);
        PinkPIDConstants translation_x_constants = new PinkPIDConstants(5, 0.0, 0.0);
        PinkPIDConstants rotation_constants = new PinkPIDConstants(3, 0.3, 0);

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
                        swerveSubsystem::setStates, mirror,
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

        new Trigger(() -> baseJoystick.getRawAxis(JoystickMap.RIGHT_TRIGGER) > 0.05)
                .whileTrue(m_intake.setCollectorPower(
                        -0.95))
                .onFalse(m_intake.setCollectorPower(0));

        // Tower
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_A)
                .whileTrue(new ShootNote(m_shooter, m_angle, m_visionSubsystem))
                .onFalse(Commands.runOnce(() -> m_shooter.setSpeed(0)));
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_B)
                .whileTrue(new ShootNoteAuto(30, -4200, m_shooter, m_angle, m_visionSubsystem));
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_X)
                .whileTrue(new ShootNoteAuto(5, 0, m_shooter, m_angle, m_visionSubsystem));
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_Y)
                .whileTrue(new ShootNoteAuto(5, 0, m_shooter, m_angle, m_visionSubsystem));

        new POVButton(baseJoystick, JoystickMap.POV_UP)
                .onTrue(Commands.runOnce(() -> climber_r2.setClimberPos(49, 49)));
        // new POVButton(baseJoystick,
        // JoystickMap.POV_LEFT).onTrue(climber_r2.setTarget(20, 20));
        // new POVButton(baseJoystick,
        // JoystickMap.POV_RIGHT).onTrue(climber_r2.setTarget(0, 0));
        new POVButton(baseJoystick, JoystickMap.POV_DOWN)
                .onTrue(Commands.runOnce(() -> climber_r2.setClimberPos(0, 0)));

        m_turret.setDefaultCommand(
                Commands.run(() -> m_turret.set(towerJoystick.getRawAxis(JoystickMap.LEFT_X_AXIS)), m_turret));

    }

    public void setupTeleop() {
        swerveSubsystem.resetGyro();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected().andThen(Commands.runOnce(() -> {
            m_shooter.stop();
            m_intake.stop();
        })).andThen(m_shooter.stopShooter());
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
