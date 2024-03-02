package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretSubsystem;

public class CollectTransferNote extends SequentialCommandGroup {
    public CollectTransferNote(Intake intake, Shooter shooter, Angle angle, TurretSubsystem turretSubsystem) {
        var prepare_shooter = new ParallelCommandGroup(angle.setAngleCommand(4), turretSubsystem.setTargetPosition(
                0));

        Command intake_command = intake.setAnglePosition(734).alongWith(
                new WaitUntilCommand(() -> intake.getAngleSupplier().getAsDouble() >= 362)
                        .andThen(prepare_shooter));

        addCommands(
                new ConditionalCommand(intake_command,
                        Commands.none(), () -> intake.noteFound() || intake.getAngleSupplier().getAsDouble() <= 50),
                intake.transferNote()
                        .alongWith(shooter.loadNoteUntilFound(0.35)),
                intake.setCollectorPower(0), intake.setAnglePosition(5));
    }
}
