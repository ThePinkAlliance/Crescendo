package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SmartLuanch extends Command {
    boolean reached_max_bottom = false;
    boolean reached_max_top = false;
    boolean is_finished = false;
    Shooter shooter;
    Timer timer;

    public SmartLuanch(Shooter shooter) {
        this.shooter = shooter;
        this.timer = new Timer();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        this.is_finished = false;
        this.reached_max_bottom = false;
        this.reached_max_top = false;

        this.timer.reset();
        this.timer.start();
        this.shooter.launch(1);
    }

    @Override
    public void execute() {
        double top_vel = this.shooter.getTopVelocity().getValueAsDouble();
        double bottom_vel = this.shooter.getBottomVelocity().getValueAsDouble();
        double top_vel_diff = Math.abs(this.shooter.getTopDesiredVelocity() - top_vel);
        double bottom_vel_diff = Math.abs(this.shooter.getBottomDesiredVelocity() - bottom_vel);

        if (top_vel_diff <= 500) {
            reached_max_top = true;
        }

        if (bottom_vel_diff <= 500) {
            reached_max_bottom = true;
        }

        if ((reached_max_bottom && reached_max_top) && (top_vel_diff >= 550 && bottom_vel_diff >= 550)) {
            is_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.setSpeed(0);
        this.timer.stop();
        this.shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return is_finished || timer.hasElapsed(1);
    }
}
