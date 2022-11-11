package com.team6647;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.team6647.subsystems.ChassisSubsystem;
import com.team6647.subsystems.PhotonCameraSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.team6647.Constants.OIConstants;
import com.team6647.utils.XboxControllerUpgrade;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
        public final ChassisSubsystem chassis = new ChassisSubsystem();

        public final PhotonCameraSubsystem photonCam = new PhotonCameraSubsystem("arducam", chassis);

        private final XboxControllerUpgrade joystick1 = new XboxControllerUpgrade(OIConstants.KDriverControllerPort,
                        0.2);

        /*
         * SendableChooser<Command> driveChooser = new SendableChooser<>();
         */

        /*
         * private final Command tankDriveCommand = new RunCommand(
         * () -> chassis.tankDrive(joystick1.getLeftY(), joystick1.getRightY()),
         * chassis);
         * 
         */

        private final Command arcadeDriveCommand = new RunCommand(() -> chassis.arcadeDrive(joystick1.getLeftY(),
                        -joystick1.getRightX()), chassis);

        /*
         * 
         * private final Command curvatureDriveCommand = new RunCommand(
         * () -> chassis.curvatureDrive(joystick1.getLeftY(), joystick1.getRightX()),
         * chassis);
         * 
         * private final Command triggerDriveCommand = new RunCommand(
         * () -> chassis.arcadeDrive(
         * (joystick1.getRightTriggerAxis() - joystick1.getLeftTriggerAxis()) * -1,
         * joystick1.getLeftX() * -1),
         * chassis);
         */

        public RobotContainer() {
                /* Set driverChooser options */
                /*
                 * driveChooser.setDefaultOption("Tank Drive", tankDriveCommand);
                 * driveChooser.addOption("Tank Drive IK", tankDriveIKCommand);
                 * driveChooser.addOption("Arcade Drive", arcadeDriveCommand);
                 * driveChooser.addOption("Arcade Drive IK", arcadeDriveIKCommand);
                 * driveChooser.addOption("Curvature Drive", curvatureDriveCommand);
                 * driveChooser.addOption("Curvature Drive IK", curvatureDriveIKCommand);
                 * driveChooser.addOption("Trigger Drive", triggerDriveCommand);
                 * SmartDashboard.putData(driveChooser);
                 */

                chassis.setDefaultCommand(arcadeDriveCommand);
                /*
                 * chassis.setDefaultCommand(chooserCommand());
                 */
                configureButtonBindings();
        }

        private void configureButtonBindings() {
                new JoystickButton(joystick1, Button.kLeftBumper.value)
                                .whileHeld(new StartEndCommand(() -> photonCam.toggleAim(), () -> photonCam.toggleAim(),
                                                photonCam));

        }

        /* Returns auto chosser selection */
        public Command getAutonomousCommand() {
                return new StartEndCommand(() -> photonCam.toggleAim(), () -> photonCam.toggleAim(),
                                photonCam);
        }

        /* Returns chooser selection */
        /*
         * public Command chooserCommand() {
         * return driveChooser.getSelected();
         * }
         */
}