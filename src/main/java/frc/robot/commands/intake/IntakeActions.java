package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intake.Intake;

public class IntakeActions {
    public static Command transferNote(double collect_position, Intake m_intake, Shooter m_shooter) {
        return new SequentialCommandGroup(
                m_intake.setAnglePosition(
                        collect_position),
                m_intake.setCollectorPower(
                        Constants.IntakeConstants.DEFAULT_COLLECT_DUTY_CYCLE))
                .alongWith(
                        m_shooter
                                .loadNoteUntilFound2(
                                        2000))
                .andThen(m_intake.setCollectorPower(
                        0));
    }
}
