// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionManager {
    private final PhotonCamera camera;
    private final double TARGET_HEIGHT_INCHES = 26.1875;

    public VisionManager() {
        this.camera = new PhotonCamera("photon");
    }

    public double getBestTargetDistance() {
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();

        return Math.sin(target.getPitch() * (Math.PI / 180)) * TARGET_HEIGHT_INCHES;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return camera.getLatestResult().getTargets();
    }
}
