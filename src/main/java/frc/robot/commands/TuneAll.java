package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.LinearInterpolationTable;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.VisionManager;
import java.util.List;

public class TuneAll extends Command {
    VisionManager m_manager;
    Shooter m_shooter;
    Angle m_angle;
    LinearInterpolationTable angleTable;

    public TuneAll(Shooter shooter, Angle angle, VisionManager manager) {
        this.m_angle = angle;
        this.m_manager = manager;
        this.m_shooter = shooter;

        List<Pair<Double, Double>> points = List.of(
                new Pair<Double, Double>(28.2, 63.55),
                new Pair<Double, Double>(
                        40.2, 54.66),
                new Pair<Double, Double>(52.2, 47.36),
                new Pair<Double, Double>(64.2, 41.45),
                new Pair<Double, Double>(76.2, 36.65),
                new Pair<Double, Double>(88.2, 32.73),
                new Pair<Double, Double>(100.2, 29.50),
                new Pair<Double, Double>(112.2, 26.80),
                new Pair<Double, Double>(124.2, 24.53),
                new Pair<Double, Double>(136.2, 22.60),
                new Pair<Double, Double>(148.2, 20.93),
                new Pair<Double, Double>(172.2, 18.22),
                new Pair<Double, Double>(196.2, 16.11),
                new Pair<Double, Double>(220.2, 14.43));

        this.angleTable = new LinearInterpolationTable(points);

        addRequirements(shooter, angle);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("desiredAngle", 0);
        SmartDashboard.putNumber("desiredRpm", 0);
    }

    @Override
    public void execute() {
        double distance = Units.metersToInches(m_manager.getBestTargetDistance());
        double estimatedAngle = this.angleTable.interp(distance);

        SmartDashboard.putNumber("desiredAngle", estimatedAngle);

        double desiredRpm = SmartDashboard.getNumber("desiredRpm", 0);
        double desiredAngle = SmartDashboard.getNumber("desiredAngle", estimatedAngle);

        this.m_angle.setAngle(desiredAngle);
        this.m_shooter.setVelocity(desiredRpm);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_shooter.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
