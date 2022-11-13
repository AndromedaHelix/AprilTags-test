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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonCameraSubsystem extends SubsystemBase {

  PhotonCamera camera;

  Notifier notifier;
  Object lock = new Object();

  private boolean aiming = false, firstRun = true;

  PIDController forwardController = new PIDController(0.1, 0, 0);

  PIDController turnController = new PIDController(0.1, 0, 0);

  ChassisSubsystem chassis;

  public PhotonCameraSubsystem(String name, ChassisSubsystem chassis) {
    camera = new PhotonCamera(name);
    camera.setPipelineIndex(0);
    this.chassis = chassis;

    notifier = new Notifier(() -> {
      synchronized (lock) {
        if (firstRun) {
          Thread.currentThread().setName("photonvision"); // TODO adjust camera name
          Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
          firstRun = false;
        }

        SmartDashboard.putBoolean("Aiming:", aiming);
        if (!aiming)
          return;

        synchronized (PhotonCameraSubsystem.this) {
          boolean hasTargets = camera.getLatestResult().hasTargets();
          SmartDashboard.putBoolean("HasTargets:", hasTargets);
          if (!hasTargets) {
            System.out.println("HAS NO TARGETS ");
            return;
          }

          var result = camera.getLatestResult();

          // ONLY GETS DISTANCE
          double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstats.cameraHeight,
              VisionConstats.targetHeight, VisionConstats.cameraPitch,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

          // Calculates the PID for the forward and turn 
          double forwardSpeed = -forwardController.calculate(range, VisionConstats.goalRange);
          double rotationSpeed = -turnController.calculate(result.getBestTarget().getPitch(), 0);

          double leftSpeed = forwardSpeed + rotationSpeed;
          double rightSpeed = forwardSpeed - rotationSpeed;

          SmartDashboard.putNumber("Forward Speed:", forwardSpeed);
          SmartDashboard.putNumber("Rotation Speed:", rotationSpeed);

          SmartDashboard.putNumber("Left Motor input speed", leftSpeed);
          SmartDashboard.putNumber("Right Motor input speed", rightSpeed);

          chassis.tankDrive(leftSpeed, -rightSpeed);
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
