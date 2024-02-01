// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Loader;

public class LoadAction extends Command {

  public enum LoadActionType{
    LAUNCH,
    LOAD,
    IDLE
  }
  
  LoadActionType m_type;
  Loader m_loader;
  double m_rpm;
  BooleanSupplier m_execute;

  /** Creates a new LoadAction. */
  public LoadAction(BooleanSupplier execute, Loader loader, LoadActionType type, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_loader = loader;
    m_type = type;
    m_rpm = rpm;
    m_execute = execute;
    addRequirements(m_loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_type == LoadActionType.LAUNCH) {
      m_loader.move(-m_rpm);
    } else if (m_type == LoadActionType.LOAD) {
      m_loader.move(m_rpm);
    } else {
      m_loader.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_loader.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_execute.getAsBoolean();
  }
}
