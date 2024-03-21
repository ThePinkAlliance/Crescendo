package frc.robot.commands.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.function.DoubleSupplier;

public class TurretVectoring extends Command {
    TurretSubsystem turret;
    VisionSubsystem vision;
    PIDController pidController;
    Timer timer;
    double error = 0;

    public TurretVectoring(TurretSubsystem turret, VisionSubsystem vision, DoubleSupplier angleSupplier) {
        this.turret = turret;
        this.vision = vision;
        this.pidController = new PIDController(.40, 0.045, 0.0);
        this.timer = new Timer();
        this.pidController.setTolerance(.5);

        addRequirements(vision, turret);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double kF = 0.0;
        double tag_angle = vision.getTargetX();
        double turret_angle = turret.getPositionDeg();
        double distance = vision.UncorrectedDistance();
        double target_pos = turret_angle - tag_angle;
        // 4 or 3.12
        double angle_compensation = distance * 3.13 / 91;

        double effort = pidController.calculate(tag_angle, 0) * -1;
        double power2 = (effort / 15) + kF;

        Logger.recordOutput("AutoLock/Effort", power2);
        Logger.recordOutput("AutoLock/Pos", target_pos);
        Logger.recordOutput("AutoLock/TagAngle", tag_angle);
        Logger.recordOutput("AutoLock/TurretPos", turret_angle);
        Logger.recordOutput("AutoLock/angle_compensation", angle_compensation);

        this.turret.set(power2);
    }

    @Override
    public void end(boolean interrupted) {
        this.turret.set(0);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint() && turret.getPower() <= 0.05 || timer.hasElapsed(1);
    }
}
