package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants.RobotType;
import org.littletonrobotics.junction.Logger;

public class CollectorEncoderv2 {
    private Encoder hexEncoder;
    private CANcoder canCoder;

    public static final int[] HEX_ENCODER_IDS = { 4, 5 };
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double PULSE_PER_REVOLUTION = 250; // Need to revisit this value!!
    public final double DISTANCE_PER_PULSE = (double) (Math.PI * WHEEL_DIAMETER) / PULSE_PER_REVOLUTION;
    private RobotType currentRobot = Constants.RobotConstants.CURRENT_ROBOT;
    private final double SENSOR_OFFSET;

    public CollectorEncoderv2() {
        if (currentRobot == RobotType.ROBOT_ONE) {
            this.hexEncoder = new Encoder(HEX_ENCODER_IDS[0], HEX_ENCODER_IDS[1]);
            SetupHexEncoder(hexEncoder, true);
            this.SENSOR_OFFSET = 0;
        } else if (currentRobot == RobotType.ROBOT_TWO) {
            this.canCoder = new CANcoder(7, "rio");
            var cancoderConfig = new CANcoderConfiguration();
            cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            cancoderConfig.MagnetSensor.MagnetOffset = 0.120;
            this.canCoder.getConfigurator().apply(cancoderConfig);
            this.SENSOR_OFFSET = 0;// -0.893;
        } else {
            this.SENSOR_OFFSET = 0;
        }
    }

    public double getPosition() {

        double position;

        if (currentRobot == RobotType.ROBOT_ONE) {
            position = (this.hexEncoder.get() + SENSOR_OFFSET);
        } else {
            position = (this.canCoder.getAbsolutePosition().getValueAsDouble() + SENSOR_OFFSET) * 100;
        }

        Logger.recordOutput("CollectorEncoder/position", position);

        return position;
    }

    public double getRawPosition() {

        double position;

        if (currentRobot == RobotType.ROBOT_ONE) {
            position = this.hexEncoder.get() + SENSOR_OFFSET;
        } else {
            position = this.canCoder.getAbsolutePosition().getValueAsDouble() + SENSOR_OFFSET;
        }

        Logger.recordOutput("CollectorEncoder/raw_position", position);

        return position;
    }

    public void reset() {
        if (currentRobot == RobotType.ROBOT_ONE) {
            this.hexEncoder.reset();
        } else {
            this.canCoder.setPosition(0);
        }
    }

    private void SetupHexEncoder(Encoder enc, boolean reverseDirection) {

        if (enc == null)
            return;
        enc.setMaxPeriod(.1);
        enc.setMinRate(10);
        System.out.println("SetupHexEncoder: Distance per Pulse: " + DISTANCE_PER_PULSE);
        enc.setDistancePerPulse(DISTANCE_PER_PULSE);
        enc.setReverseDirection(reverseDirection);
        enc.setSamplesToAverage(7);
        enc.reset();
    }

}
