import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathing.events.ChoreoEvent;
import frc.lib.pathing.events.ChoreoEventHandler;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class EventsTest {
    private ChoreoEventHandler handler;
    private SwerveSubsystem swerveSubsystem;

    @BeforeEach
    public void before() {
        // HAL.initialize(500, 0);

        // ChoreoEvent[] events = ChoreoEvent
        // .createArray(new ChoreoEvent(Commands.print("hello"), new Translation2d(0,
        // 1)));

        // this.swerveSubsystem = new
        // SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
        // this.handler = new ChoreoEventHandler(events);
    }

    @Test
    public void eventExecuteOnWaypoint() {
        // this.swerveSubsystem.resetPose(new Pose2d(0, 1, new Rotation2d()));
        // Optional<ChoreoEvent> event =
        // this.handler.compute(this.swerveSubsystem.getCurrentPose());

        // assertEquals(true, event.isPresent());
    }

    @Test
    public void eventExecuteOffWaypoint() {
        // this.swerveSubsystem.resetPose(new Pose2d(0, 2, new Rotation2d()));
        // Optional<ChoreoEvent> event =
        // this.handler.compute(this.swerveSubsystem.getCurrentPose());

        // assertEquals(false, event.isPresent());
    }
}
