// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/* 
 * Modify the motor ID constants using the appropiate software (Phoenix Tuner, Rev Client, etc.)
 */

public final class Constants {
    public static final class ChassisConstants {
        public static final int frontRight = 1;
        public static final int frontLeft = 2; // 5
        public static final int backLeft = 3; // 8
        public static final int backRight = 4; // 9
    }

    public static final class OIConstants {
        public static double multiplier = 0;
        public static final int KDriverControllerPort = 0;
    }

    public static final class VisionConstats {
        public static final double cameraHeight = Units.inchesToMeters(2);
        public static final double targetHeight = Units.inchesToMeters(65); // TODO modify
        public static final double cameraPitch = Units.degreesToRadians(0);

        public static final double goalRange = Units.inchesToMeters(65);
    }

    public static final class DriveConstants {
        public static final double ksVolts = 0.1519;
        public static final double kvVoltSecondsPerMeter = 0.63; // 1.6277
        public static final double kaVoltSecondsSquaredPerMeter = 0.03; // 0.22002
        public static final double kpDriveVelocity = 0.38; // 2.0958

        public static final double kTrackWidthMeters = Units.inchesToMeters(22);
        public static final DifferentialDriveKinematics kDrivekinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);

        public static final double kMaxVelocity = 3;
        public static final double kMaxAcceleration = 3;

        public static final double kramseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kGearRatio = 8.45;
        public static final double kWheelRadius = 3;

        public static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius)) * 10));
    }
}
