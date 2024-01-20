package frc.lib.pathing.events;

import java.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ChoreoEventHandler {
    private Queue<ChoreoEvent> queue;
    private Translation2d previousDifference;
    private double differenceTolerance;

    public ChoreoEventHandler(ChoreoEvent... events) {
        if (events == null) {
            events = new ChoreoEvent[] {};
        }

        this.queue = new LinkedList<ChoreoEvent>(List.of(events));
        this.differenceTolerance = 0.06;
    }

    public Optional<ChoreoEvent> compute(Pose2d _robot_pose) {
        ChoreoEvent event = this.queue.peek();

        if (event != null && !queue.isEmpty()) {
            Translation2d event_pos = event.getPosition();
            Translation2d difference = event_pos.minus(_robot_pose.getTranslation());

            if (Math.abs(difference.getNorm()) <= differenceTolerance) {
                try {
                    queue.remove();
                } catch (NoSuchElementException err) {
                    System.out.println("Queue Empty");
                }

                return Optional.of(event);
            }

            this.previousDifference = difference;
        }

        return Optional.empty();
    }
}
