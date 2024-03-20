package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class CollectNoteV2 extends SequentialCommandGroup {
    public CollectNoteV2(Intake intake, Shooter shooter, Angle angle, TurretSubsystem turretSubsystem) {
        var prepare_shooter = new ParallelCommandGroup(angle.setAngleCommand(Constants.AngleConstants.IDLE_ANGLE),
                turretSubsystem.setTargetPosition(
                        0));

        addCommands(
                intake.setAnglePosition(Constants.IntakeConstants.COLLECT_FLOOR_POS).alongWith(
                        new WaitUntilCommand(
                                () -> intake.getCollectorPosition() >= Constants.IntakeConstants.COLLECT_MID_POS)
                                .andThen(prepare_shooter)),
                intake.collectUntilFound(Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE),
                intake.goToTransfer()
                        .alongWith(shooter.loadNoteUntilFound2(2000)),
                intake.setCollectorPower(0), intake.setAnglePosition(5));
    }
}
