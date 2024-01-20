package frc.lib.pathing.events;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ChoreoEvent {
    private Command command;
    private Translation2d position;

    public ChoreoEvent(Command _command, Translation2d _transform2d) {
        this.command = _command;
        this.position = _transform2d;
    }

    public void schedule() {
        this.command.schedule();
    }

    public Command getCommand() {
        return this.command;
    }

    public Translation2d getPosition() {
        return this.position;
    }

    public static ChoreoEvent[] createArray(ChoreoEvent... events) {
        return events;
    }
}
