package frc.robot.commands.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
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
        this.pidController = new PIDController(.1, 0.01, 0.0);

        addRequirements(vision, turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double tag_angle = vision.getTargetX();
        double turret_angle = turret.getPositionDeg();
        double target_pos = turret_angle + tag_angle;
        double effort = this.pidController.calculate(this.turret.getPosition(),
                target_pos);
        Logger.recordOutput("Turret/Effort", effort);
        Logger.recordOutput("AutoLock", target_pos);
        // this.turret.set(effort);
    }

    @Override
    public void end(boolean interrupted) {
        this.turret.set(0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
