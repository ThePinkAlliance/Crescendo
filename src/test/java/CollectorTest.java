import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class CollectorTest {
    static Intake intake;

    @BeforeAll
    public static void before() {
        HAL.initialize(500, 0);

        intake = new Intake();
    }

    public void sam(double x, double a) {
        // double r = 0.0005 * (x * x * x) - 0.1032 * (x * x) + 7.1735 * x - 112.4;
        // double r = Math.pow(0.0005 * x, 3) - Math.pow(0.1032 * x, 2) + 7.1735 * x -
        // 112.4;
        // double r = 103.6 * Math.pow(Math.E, -0.014 * x);
        // double r = 0.0019 * (x * x) - 0.7904 * x + 85.839;
        double r = -0.4991 * x + 74.924;

        assertEquals(a, r, .5);
    }

    @Test
    public void testPID() {
        double t = -0.014 * 89;
        double e = Math.pow(281.60552, t);
        double d = 77.3;

        // 0.0005x3 - 0.1032x2 + 7.1735x - 112.4

        double x1 = 0.0005 * (d * d * d) - 0.1032 * (d * d) + 7.1735 * d - 112.4;

        // 0.0002x3 - 0.0441x2 + 2.6755x

        double x2 = 0.0002 * (d * d * d) - 0.0441 * (d * d) + 2.6755 * d;

        // double x3 = 103.6 * Math.E ^ (-0.014 * d);

        double target_angle = 0.0145 * Math.pow(d, 2) - 2.5546 * d + 143.1;

        assertEquals(32, target_angle, .5);

        System.out.println(e);
        // sam(84.4, 26);
    }

    @Test
    public void yeaaa() {
        double distance = 90;
        double target_angle = -8; // (0.0016 * Math.pow(distance, 2) - 0.8857 * distance + 84.774 + 4);

        if (distance <= 100 && distance >= 0) {
            if (target_angle <= 0) {
                target_angle = 0;
            }

            assertEquals(0, target_angle);
        }
    }

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
