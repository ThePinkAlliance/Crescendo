// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.JoystickMap;
import frc.lib.PinkPIDConstants;
import frc.lib.pathing.ChoreoUtil;
import frc.lib.pathing.events.ChoreoEvent;
import frc.robot.commands.autos.LeaveZone;
import frc.robot.commands.autos.StealMidBlueStatic;
import frc.robot.commands.autos.StealMidRedMoving;
import frc.robot.commands.autos.StealMidRedStatic;
import frc.robot.commands.autos.SweepNotesBlue;
import frc.robot.commands.autos.SweepNotesMiniBlue;
import frc.robot.commands.autos.SweepNotesMiniRed;
import frc.robot.commands.autos.SweepNotesRed;
import frc.robot.commands.autos.TwoNoteBlue;
import frc.robot.commands.autos.TwoNoteRed;
import frc.robot.commands.climber.ClimbSequence;
import frc.robot.commands.shooter.ShootNoteAuto;
import frc.robot.commands.shooter.ShootNoteTargetVisible;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.ClimberR2;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.commands.intake.AmpShot;
import frc.robot.commands.intake.CollectNoteV2;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;
import frc.robot.commands.shooter.ShootNoteTargetVisible;

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

        this.chooser.addOption("Blue Sweep Mini Left",
                SweepNotesMiniBlue.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));
        this.chooser.addOption("Blue Steal Mid Shoot Static",
                StealMidBlueStatic.getLeft(swerveSubsystem, m_turret, m_intake, m_angle, m_visionSubsystem, m_shooter));

        this.chooser.addOption("None", Commands.none());
        this.chooser.addOption("Leave Zone Left", LeaveZone.leftZone(swerveSubsystem));

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
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_B).onTrue(new AmpShot(m_intake, swerveSubsystem));
        new JoystickButton(baseJoystick, JoystickMap.RIGHT_BUMPER)
                .whileTrue(new CollectNoteV2(m_intake, m_shooter, m_angle, m_turret)).onFalse(
                        m_intake.setCollectorPower(0));
        new JoystickButton(baseJoystick, JoystickMap.LEFT_BUMPER).onTrue(m_intake.setAnglePosition(0));
        new Trigger(() -> baseJoystick.getRawAxis(JoystickMap.RIGHT_TRIGGER) > 0.05)
                .whileTrue(m_intake.setCollectorPower(
                        -0.95))
                .onFalse(m_intake.setCollectorPower(0));

        new POVButton(baseJoystick, JoystickMap.POV_UP)
                .onTrue(climber_r2.travelToClimberPos(49, 49));
        new POVButton(baseJoystick, JoystickMap.POV_DOWN)
                .onTrue(climber_r2.travelToClimberPos(0, 0));

        //Climber Sequence - assumes driver has already extended the climber and position the hooks over the chain
        new JoystickButton(baseJoystick, JoystickMap.BUTTON_A)
                .onTrue(new ClimbSequence(m_intake, m_turret, climber_r2));

        // Tower
        //A BUTTON without the conditional check on a visible apriltag
        // new JoystickButton(towerJoystick, JoystickMap.BUTTON_A)
        //         .onTrue(new ParallelCommandGroup(
        //                 new TurretVectoring(m_turret, m_visionSubsystem, () -> swerveSubsystem.getHeading()),
        //                 new ShootNote(m_shooter, m_angle, m_turret,
        //                         () -> m_visionSubsystem.UncorrectedDistance()))
        //                 .andThen(
        //                         m_turret.setTargetPositionRaw(0)
        //                 ));
        
        //Alternative to non-target visible method
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_A).onTrue(new ShootNoteTargetVisible(
                m_shooter, m_angle, m_turret, m_visionSubsystem, swerveSubsystem, 
                () -> m_visionSubsystem.UncorrectedDistance()).andThen(m_turret.setTargetPositionRaw(0)));

        new JoystickButton(towerJoystick, JoystickMap.BUTTON_Y)
                .whileTrue(m_shooter.loadNoteUntilFound2(1000)).onFalse(m_shooter.stopShooter());
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_X)
                .whileTrue(new ShootNoteAuto(45, -2800, m_shooter, m_angle,
                        m_visionSubsystem).compose());
        new JoystickButton(towerJoystick, JoystickMap.BUTTON_B)
                .whileTrue(new ShootNoteAuto(48, -3800, m_shooter, m_angle,
                        m_visionSubsystem).compose());

        climber_r2.setDefaultCommand(Commands.run(() -> {
            double left = towerJoystick.getRawAxis(JoystickMap.LEFT_Y_AXIS) * -1;
            double right = towerJoystick.getRawAxis(JoystickMap.RIGHT_Y_AXIS) * -1;

            if (Math.abs(right) > 0.5) {
                right = 0.5;
            }

            if (Math.abs(left) > 0.5) {
                left = 0.5;
            }

            this.climber_r2.testPower(right, left);
        }, climber_r2));
        
        new POVButton(towerJoystick, JoystickMap.POV_LEFT).onTrue(m_turret.setTargetPosition(0));
        new POVButton(towerJoystick, JoystickMap.POV_RIGHT).onTrue(m_turret.setTargetPosition(180));
        
    }

    public void onDisabled() {
        this.m_turret.setBrakeMode(IdleMode.kCoast);
        m_shooter.stop();
        m_intake.stop();
        m_shooter.stopShooter().initialize();
    }

    public void setupTeleop() {
        this.m_turret.setBrakeMode(IdleMode.kBrake);
        swerveSubsystem.resetGyro();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected().handleInterrupt(() -> {
            m_shooter.stop();
            m_intake.stop();
            m_shooter.stopShooter().initialize();
        });
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
