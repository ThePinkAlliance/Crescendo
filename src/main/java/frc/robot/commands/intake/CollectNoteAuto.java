// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectNoteAuto extends SequentialCommandGroup {
    /** Creates a new CollectNote. */
    public CollectNoteAuto(Intake intake, Shooter shooter, TurretSubsystem turret, Angle angle) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                // Set turret zero here.
                intake.deployCollector(.40).alongWith(turret.setTargetPosition(
                        0)),
                intake.collectUntilFound(.85),
                angle.setAngleCommand(0),
                intake.goToTransfer().alongWith(shooter.loadNoteUntilFound(0.3)),
                intake.setCollectorPower(0),
                intake.stowCollector());
    }
}
