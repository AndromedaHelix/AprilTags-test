// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.team6647.Constants.VisionConstats;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonCameraSubsystem extends SubsystemBase {

  PhotonCamera camera;

  private final Object lock = new Object();
  private final Notifier notifier;
  private boolean aiming = false, firstRun = true;

  PIDController forwardController = new PIDController(0.1, 0, 0);

  PIDController turnController = new PIDController(0.1, 0, 0);

  public PhotonCameraSubsystem(String name, ChassisSubsystem chassis) {
    camera = new PhotonCamera(name);
    notifier = new Notifier(() -> {
      synchronized (lock) {
        if (firstRun) {
          Thread.currentThread().setName("photonvision");
          Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
          firstRun = false;
        }

        if (!aiming)
          return;

        synchronized (PhotonCameraSubsystem.this) {
          if (!camera.getLatestResult().hasTargets())
            return;

          var result = camera.getLatestResult();

          // ONLY GETS DISTANCE
          double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstats.cameraHeight,
              VisionConstats.targetHeight, VisionConstats.cameraPitch,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

          /* Calculates the PID */
          double forwardSpeed = -forwardController.calculate(range, VisionConstats.goalRange);

          double rotationSpeed = -turnController.calculate(result.getBestTarget().getPitch(), 0);

          chassis.arcadeDrive(forwardSpeed, rotationSpeed);
        }
      }
    });
    notifier.startPeriodic(0.01);
  }

  @Override
  public void periodic() {}

  public void toggleAim() {
    aiming = !aiming;
  }
}
