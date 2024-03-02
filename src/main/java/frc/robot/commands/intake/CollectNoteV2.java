package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.intake.Intake;

public class CollectNoteV2 extends SequentialCommandGroup {
    public CollectNoteV2(Intake intake, Shooter shooter, Angle angle, TurretSubsystem turretSubsystem) {
        var prepare_shooter = new ParallelCommandGroup(angle.setAngleCommand(4), turretSubsystem.setTargetPosition(
                0));

        addCommands(
                intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS).alongWith(
                        new WaitUntilCommand(
                                () -> intake.getCollectorPosition() >= Constants.IntakeConstants.COLLECT_MID_POS)
                                .andThen(prepare_shooter)),
                intake.collectUntilFound(.85), intake.goToTransfer()
                        .alongWith(shooter.loadNoteUntilFound(0.35)),
                intake.setCollectorPower(0), intake.setAnglePosition(5));
    }
}
