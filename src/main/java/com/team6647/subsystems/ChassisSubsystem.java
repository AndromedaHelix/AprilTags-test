// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6647.Constants.ChassisConstants;
import com.team6647.Constants.DriveConstants;

/*
 * This is the chassis subsystem. 
 * Change the motor type to the used motor in your robot
 */
public class ChassisSubsystem extends SubsystemBase {

  // Create motor objects
  private static CANSparkMax frontLeft = new CANSparkMax(ChassisConstants.frontLeft, MotorType.kBrushless);
  private static CANSparkMax frontRight = new CANSparkMax(ChassisConstants.frontRight, MotorType.kBrushless);
  private static CANSparkMax rearLeft = new CANSparkMax(ChassisConstants.backLeft, MotorType.kBrushless);
  private static CANSparkMax rearRight = new CANSparkMax(ChassisConstants.backRight, MotorType.kBrushless);

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(frontLeft, rearLeft);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(frontRight, rearRight);

  private static final RelativeEncoder leftEncoder = frontLeft.getEncoder();
  private static final RelativeEncoder rightEncoder = frontRight.getEncoder();

  private static DifferentialDrive chassis;

  private double leftSpeed, rightSpeed;

  public static final AHRS navx = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry odometry;

  private Field2d field = new Field2d();

  // Constructor method, is called when object is created
  public ChassisSubsystem() {
    /* Chassis initialization instructions */
    frontLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();

    rearLeft.restoreFactoryDefaults();
    rearRight.restoreFactoryDefaults();

    setMotorsIdleMode(IdleMode.kCoast);

    leftControllerGroup.setInverted(true);
    rightControllerGroup.setInverted(false);

    chassis = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

     /* Odometry and navx instructions */
    resetEncoders();

    leftEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    rightEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);

    leftEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);

    navx.reset();
    navx.calibrate();

    odometry = new DifferentialDriveOdometry(navx.getRotation2d());

    resetOdometry(new Pose2d());

    SmartDashboard.putData("Field:", field);
  }

  @Override
  public void periodic() {
    publishData();
    updateRotation2D();
    field.setRobotPose(getPose());
  }

  /*
   * Odometry functions
   */

  /* Updates odometry rotation */
  public void updateRotation2D() {
    odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /* Returns odometry pose in meters */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /*
   * Encoders functions
   */

  /* Resets encoder position */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /* Returns left encoder position */
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  /* Returns right encoder position */
  public double getRightEncoderPositions() {
    return -rightEncoder.getPosition();
  }

  /* Returns left encoder velocity */
  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  /* Returns right encoder velocity */
  public double getRightEncoderVelocity() {
    return -rightEncoder.getVelocity();
  }

  /* Returns average between encoder positions */
  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPositions()) / 2);
  }

  /* Returns left encoder object */
  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  /* Returns right encoder object */
  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  /*
   * NAVX functions
   */

  /* Returns navx heading */
  private double getHeading() {
    return -navx.getRotation2d().getDegrees();
  }

  /* Gets navx turn rate */
  public double getTurnRate() {
    return navx.getRate();
  }

  /* Resets odometry position */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, navx.getRotation2d());
  }

  /* Resets navx heading */
  public static void zeroHeading() {
    navx.calibrate();
    navx.reset();
  }

  /* Returns navx object */
  public Gyro getNavx() {
    return navx;
  }

  /*
   * Differential Drive functions
   */

  /* Returns wheel speeds */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  /* Sets motor voltage */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllerGroup.setVoltage(leftVolts);
    rightControllerGroup.setVoltage(rightVolts);
    chassis.feed();
  }

  /* Sets differentialDrive max output */
  public void setMaxOutput(double maxOutput) {
    chassis.setMaxOutput(maxOutput);
  }

  /* Set motor idle mode */
  public void setMotorsIdleMode(IdleMode idleMode) {
    frontLeft.setIdleMode(idleMode);
    frontRight.setIdleMode(idleMode);
    rearLeft.setIdleMode(idleMode);
    rearRight.setIdleMode(idleMode);
  }

  // Publish to SmartDashboard for debugging
  private void publishData() {
    SmartDashboard.putNumber("LeftSpeed", leftSpeed);
    SmartDashboard.putNumber("RightSpeed", rightSpeed);
    SmartDashboard.putNumber("Left Encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder", getRightEncoderPositions());
    SmartDashboard.putNumber("Navx heading", getHeading());
    SmartDashboard.putNumber("Navx turn rate", getTurnRate());
  }

  /* Different drive modes */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    chassis.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double linearSpeed, double rotSpeed) {
    this.leftSpeed = linearSpeed;
    this.rightSpeed = rotSpeed;
    chassis.arcadeDrive(linearSpeed, rotSpeed);
  }

  public void curvatureDrive(double rightSpeed, double leftSpeed) {
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    chassis.curvatureDrive(leftSpeed, rightSpeed, true);
  }
}
