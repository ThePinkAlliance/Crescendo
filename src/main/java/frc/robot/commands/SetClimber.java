// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberSide;

public class SetClimber extends Command {
  /** Creates a new ResetClimber. */
  private int m_lposition;
  private int m_rposition;
  private Climber m_climber;
  private Climber.ClimberSide m_side;
  public SetClimber(Climber climber, int lposition,int rposition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lposition = lposition;
    m_rposition = rposition;
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_climber.setLeftTargetPosition2(m_lposition);
    System.out.println("left set position " + m_lposition);
  
    m_climber.setRightTargetPosition2(m_rposition);
    System.out.println("right set position " + m_rposition);
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
