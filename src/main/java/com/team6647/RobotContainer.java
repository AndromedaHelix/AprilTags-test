package com.team6647;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.team6647.subsystems.ChassisSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.team6647.Constants.OIConstants;
import com.team6647.utils.XboxControllerUpgrade;

public class RobotContainer {
        public final ChassisSubsystem chassis = new ChassisSubsystem();

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
        }

        /* Returns auto chosser selection */
        public Command getAutonomousCommand() {
                return null;
        }

        /* Returns chooser selection */
        /*
         * public Command chooserCommand() {
         * return driveChooser.getSelected();
         * }
         */
}