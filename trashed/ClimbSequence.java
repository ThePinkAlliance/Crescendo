// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ClimberR2;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {

    /** Creates a new ShootNote. */
    public ClimbSequence(Intake m_intake, TurretSubsystem m_turret, ClimberR2 m_climber) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            m_intake.setAnglePosition(IntakeConstants.COLLECT_FLOOR_POS),
            m_turret.setTargetPosition(0),
            m_climber.travelToClimberPos(0, 0));
    }
}
