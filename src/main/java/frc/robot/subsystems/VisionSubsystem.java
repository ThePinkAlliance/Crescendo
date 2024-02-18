// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
    private final DoubleSubscriber target_y_subscriber;

    /** Creates a new Vision. */
    public VisionSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        this.botpose_subscriber = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
        this.target_id_subscriber = table.getIntegerTopic("tid").subscribe(0);
        this.target_x_subscriber = table.getDoubleTopic("tx").subscribe(0);
        this.target_y_subscriber = table.getDoubleTopic("ty").subscribe(0);

    }

    public Translation3d getTranslation() {
        double[] data = this.botpose_subscriber.get();

        return new Translation3d(data[0], data[1], data[2]);
    }

    public double getClosestTargetDistance() {
        double correction_factor = 0.2992125984;
        double distance = 56 / Math.tan((25 + this.target_y_subscriber.get()) * (3.14 / 180));

        distance = distance - (distance * correction_factor);

        Logger.recordOutput("Closest Target Distance", distance);
        Logger.recordOutput("Closest Target Correction Factor", correction_factor);

        return distance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Get the pose of apriltag on center of speaker red
        Pose3d tag_pose = Constants.FieldConstants.layout.getTagPose(4).get();

        Logger.recordOutput("Robot-Translation", getTranslation());
        Logger.recordOutput("Robot-Pose", new Pose2d(getTranslation().toTranslation2d(), Rotation2d.fromDegrees(0)));
    }
}
