import edu.wpi.first.math.controller.PIDController;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class CollectorTest {
    @Test
    public void intake() {
        double pos = 0;
        double angleFF = .1;
        PIDController anglePidController = new PIDController(1, 0, 0);
        double measurement = 130;

        double effort = anglePidController.calculate(measurement,
                pos)
                + (angleFF * Math.sin(measurement * (1 / 734)));

        Logger.recordOutput("Intake/Control Effort 2", effort);

        // scale control effort to a ratio to make it useable with voltage control.
        effort = effort * 0.0013623978;

        System.out.println(effort);
    }
}
