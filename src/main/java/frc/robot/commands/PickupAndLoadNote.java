// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.JoystickMap;
import frc.robot.commands.shooter.ShootAction;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAndLoadNote extends SequentialCommandGroup {
  /** Creates a new PickupAndLoadNote. */
  public PickupAndLoadNote(Intake intake, Shooter shooter, Angle angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // new JoystickButton(baseJoystick, JoystickMap.BUTTON_A).whileTrue(m_intake.setCollectorSpeed2(.85));
    //     new JoystickButton(baseJoystick, JoystickMap.LEFT_BUMPER).whileTrue(m_intake.stowCollector());
    //     new JoystickButton(baseJoystick, JoystickMap.RIGHT_BUMPER).whileTrue(m_intake.deployCollector());
    //     new JoystickButton(baseJoystick, JoystickMap.BUTTON_Y).whileTrue(m_intake.goToTransfer());
    addCommands(
      intake.setCollectorSpeed2(.85),
      intake.goToTransfer().alongWith(shooter.loadNote(0.30)),
      intake.setCollectorPower(0),
      intake.stowCollector(),
      shooter.rampUp2(-4200),
      angle.setAngleCommand(-30),
      shooter.launchNote2()
      
      //new ShootAction(4200, -30, shooter, angle)
     );
  }
}
