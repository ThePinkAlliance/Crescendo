package frc.lib.pathing.events;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ChoreoEvent {
    private Command command;
    private double timestamp;

    public ChoreoEvent(Command _command, double _timestamp) {
        this.command = _command;
        this.timestamp = _timestamp;
    }

    public void schedule() {
        this.command.schedule();
    }

    public Command getCommand() {
        return this.command;
    }

    public double getExecTime() {
        return this.timestamp;
    }

    public static ChoreoEvent[] createArray(ChoreoEvent... events) {
        return events;
    }
}
