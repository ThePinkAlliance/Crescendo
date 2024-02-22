// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.JoystickMap;
import frc.robot.commands.shooter.ShootAction;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAndLoadNote extends SequentialCommandGroup {
    VisionSubsystem visionSubsystem;

    /** Creates a new PickupAndLoadNote. */
    public PickupAndLoadNote(Intake intake, Shooter shooter, Angle angle, VisionSubsystem visionSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        // new JoystickButton(baseJoystick,
        // JoystickMap.BUTTON_A).whileTrue(m_intake.setCollectorSpeed2(.85));
        // new JoystickButton(baseJoystick,
        // JoystickMap.LEFT_BUMPER).whileTrue(m_intake.stowCollector());
        // new JoystickButton(baseJoystick,
        // JoystickMap.RIGHT_BUMPER).whileTrue(m_intake.deployCollector());
        // new JoystickButton(baseJoystick,
        // JoystickMap.BUTTON_Y).whileTrue(m_intake.goToTransfer());

        this.visionSubsystem = visionSubsystem;

        addCommands(
                Commands.runOnce(() -> angle.setAngleNew(0)),
                intake.collectUntilFound(.85),
                intake.goToTransfer().alongWith(shooter.loadNoteUntilFound(0.30)),
                intake.setCollectorPower(0),
                intake.stowCollector(),
                // shooter.rampUp2(-4200),
                // go_to_angle(angle),
                // shooter.launchNote2(),
                // angle.setAngleCommandNew(0),
                Commands.runOnce(() -> shooter.stop()));
    }

    public Command go_to_angle(Angle angle) {
        return new FunctionalCommand(() -> angle.setAngleNew(getAngle()),
                () -> {
                }, (i) -> {
                }, () -> angle.getControlError() <= 1 && angle.getSpeed() <= 0.05, angle);
    }

    public double getAngle() {
        double distance = visionSubsystem.getClosestTargetDistance();
        double desiredAngle = Math.atan2(89 - 15, distance - 16) * (180 / Math.PI);

        return desiredAngle;
    }
}
