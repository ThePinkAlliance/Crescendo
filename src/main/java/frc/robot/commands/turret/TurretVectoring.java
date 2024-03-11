package frc.robot.commands.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class TurretVectoring extends Command {
    TurretSubsystem turret;
    VisionSubsystem vision;
    PIDController pidController;

    public TurretVectoring(TurretSubsystem turret, VisionSubsystem vision) {
        this.turret = turret;
        this.vision = vision;
        this.pidController = new PIDController(.1, 0.0, 0.0);

        addRequirements(vision, turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double tag_angle = vision.getTargetX();
        double target_pos = tag_angle;
        double effort = this.pidController.calculate(this.turret.getPosition(),
                target_pos);
        Logger.recordOutput("Turret/Effort", effort);
        this.turret.set(effort);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
