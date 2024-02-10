// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberSide;

public class SetClimber extends Command {
  /** Creates a new ResetClimber. */
  private int m_position;
  private ClimberSubsystem m_climber;
  private ClimberSubsystem.ClimberSide m_side;
  public SetClimber(ClimberSubsystem climber, ClimberSubsystem.ClimberSide side, int position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_position = position;
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
    if (m_side == ClimberSubsystem.ClimberSide.LEFT) {
      m_climber.setLeftTargetPosition(m_position);
      System.out.println("left set position " + m_position);
    } else if (m_side == ClimberSubsystem.ClimberSide.RIGHT) {
      m_climber.setRightTargetPosition(m_position);
      System.out.println("right set position " + m_position);
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
