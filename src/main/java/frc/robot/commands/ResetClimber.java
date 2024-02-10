// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberSide;

public class ResetClimber extends Command {
  /** Creates a new ResetClimber. */
  private Climber m_climber;
  private Climber.ClimberSide m_side;
  public ResetClimber(Climber climber, Climber.ClimberSide side) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_side = side;
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_side == Climber.ClimberSide.LEFT) {
      m_climber.leftReset();
      System.out.println("left set position 0 ");
    } else if (m_side == Climber.ClimberSide.RIGHT) {
      m_climber.rightReset();
      System.out.println("right set position 0 ");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
