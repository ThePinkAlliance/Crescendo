// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final DoubleArraySubscriber botpose_subscriber;
    private final IntegerSubscriber target_id_subscriber;
    private final DoubleSubscriber target_x_subscriber;
    private final DoubleSubscriber target_latency_subscriber;
    private final DoubleSubscriber capture_latency_subscriber;
    private final DoubleSubscriber target_y_subscriber;
    private final DoubleSubscriber target_area_subscriber;
    private Matrix<N3, N1> correction_matrix;

    private final double mounted_angle;

    /** Creates a new Vision. */
    public VisionSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        this.botpose_subscriber = table.getDoubleArrayTopic("botpose_wpired").subscribe(null);
        this.target_id_subscriber = table.getIntegerTopic("tid").subscribe(0);
        this.target_x_subscriber = table.getDoubleTopic("tx").subscribe(0);
        this.target_y_subscriber = table.getDoubleTopic("ty").subscribe(0);
        this.capture_latency_subscriber = table.getDoubleTopic("tc").subscribe(0);
        this.target_latency_subscriber = table.getDoubleTopic("tl").subscribe(0);
        this.target_area_subscriber = table.getDoubleTopic("ta").subscribe(0);

        this.correction_matrix = VecBuilder.fill(0, 0, 0);
        this.mounted_angle = 22.5;
    }

    public Optional<Translation3d> getTranslation() {
        double[] data = this.botpose_subscriber.get();

        if (data != null) {
            double botpose_x = data[0];
            double botpose_y = data[1];
            double botpose_z = data[2];

            double corrected_x = botpose_x - (botpose_x * correction_matrix.get(0, 0));
            double corrected_y = botpose_y - (botpose_y * correction_matrix.get(1, 0));
            double corrected_z = botpose_z - (botpose_z * correction_matrix.get(2, 0));

            return Optional.of(new Translation3d(corrected_x, corrected_y, corrected_z));
        }

        return Optional.empty();
    }

    private double correctLimelightDistance(double uncorrected_distance) {
        return (1.0002 * uncorrected_distance) - 38.217;
    }

    public double getClosestTargetDistance() {
        // Using tag 4 height (56)
        double distance = 56 / Math.tan((mounted_angle + this.target_y_subscriber.get()) * (3.14 / 180));

        return correctLimelightDistance(distance);
    }

    public Optional<Double> getTagDistance(int id) {
        Optional<Double> distance = getUncorrectedTagDistance(id);

        if (distance.isPresent()) {
            return Optional.of(correctLimelightDistance(distance.get()));
        }

        return Optional.empty();
    }

    public Optional<Double> getUncorrectedTagDistance(int id) {
        Optional<Pose3d> pose = Constants.FieldConstants.layout.getTagPose(id);
        double visible_tag_id = this.target_id_subscriber.get();

        if (pose.isPresent() && visible_tag_id == id) {
            Pose3d pose3d = pose.get();
            double height = Units.metersToInches(pose3d.getZ());
            double distance = height / Math.tan((mounted_angle + this.target_y_subscriber.get()) * (3.14 / 180));

            return Optional.of(distance);
        }

        return Optional.empty();
    }

    public double UncorrectedDistance() {
        // using tag 4 height (56)
        return 56 / Math.tan((mounted_angle + this.target_y_subscriber.get()) * (3.14 / 180));
    }

    public double getVisionLatency() {
        return (target_latency_subscriber.get() / 1000) - (capture_latency_subscriber.get() / 1000);
    }

    public double getTargetX() {
        return this.target_x_subscriber.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Get the pose of apriltag on center of speaker red
        Pose3d tag_pose = Constants.FieldConstants.layout.getTagPose(4).get();
        var translation = getTranslation();

        if (translation.isPresent()) {
            Translation2d translation2d = translation.get().toTranslation2d();
            Logger.recordOutput("Robot-Translation", translation2d);
            Logger.recordOutput("Speaker-Red-Tag", tag_pose.toPose2d());
            Logger.recordOutput("Robot-Pose",
                    new Pose2d(getTranslation().get().toTranslation2d(), Rotation2d.fromDegrees(0)));
        }

        Logger.recordOutput("Closest Target Distance", getClosestTargetDistance());
        Logger.recordOutput("Uncorrected Distance", UncorrectedDistance());
    }
}
