// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.Demacia.utils.Motors.TalonConfig;

import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Demacia.utils.Sensors.CancoderConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *c
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static final double gyroOffset = 0.32 ; // Gyro offset for blue alliance in degrees
  // public static final double gyroOffsetRed = 183.2; // Gyro offset for red alliance in degrees

  public static final Camera CAMERA_POSITION = new Camera("tag", new Translation3d(0.358, -0.06, 0.646), -50.0, 0.0, "reef");
  
  public static final double MIN_CROP = 0.5; // Minimum crop value
  public static final double MAX_CROP = 0.6; // Maximum crop value
  public static final double CROP_CONSTANT = 0.35; // Crop constant for distance calculation
  public static final double PREDICT_Y = 0.15; // Y prediction for latency compensation in meters
  public static final double PREDICT_X = -0.1; // X prediction for latency compensation in meters
  public static final double PREDICT_OMEGA = 0.2; // Omega prediction for latency compensation in radians

  public static final double WORST_RELIABLE_DIST = 4; //max distance for reliable pose estimation in meters
  public static final double BEST_RELIABLE_DIST = 1; // best distance for reliable pose estimation in meters

  //height of every tag type from the ground in meters
  private static double BARGE_TAG_HIGHT = inchToMeter(73.54);
  private static double REEF_TAG_HIGHT = inchToMeter(12.13);
  private static double STATION_TAG_HIGHT = inchToMeter(58.50);
  private static double SIDE_TAG_HIGHT = inchToMeter(51.25);

        /**
         * Array of AprilTag field locations.
         * Each value is the (x, y) position of the center of the tag on the field in meters.
         * Index corresponds to AprilTag ID (0 is unused).
         */
  public static Translation2d[] O_TO_TAG = { 
    new Translation2d(0, 0), // 0;
    new Translation2d(inchToMeter(657.37), inchToMeter(25.80)), // 1
    new Translation2d(inchToMeter(657.37), inchToMeter(291.20)), // 2
    new Translation2d(inchToMeter(455.15), inchToMeter(317.15)), // 3
    new Translation2d(inchToMeter(365.20), inchToMeter(241.64)), // 4
    new Translation2d(inchToMeter(365.20), inchToMeter(75.39)), // 5
    new Translation2d(inchToMeter(530.49), inchToMeter(130.17)), // 6
    new Translation2d(inchToMeter(546.87), inchToMeter(158.50)), // 7
    new Translation2d(inchToMeter(530.49), inchToMeter(186.83)), // 8
    new Translation2d(inchToMeter(497.77), inchToMeter(186.83)), // 9
    new Translation2d(inchToMeter(481.39), inchToMeter(158.50)), // 10
    new Translation2d(inchToMeter(497.77), inchToMeter(130.17)), // 11
    new Translation2d(inchToMeter(33.51), inchToMeter(25.80)), // 12
    new Translation2d(inchToMeter(33.51), inchToMeter(291.20)), // 13
    new Translation2d(inchToMeter(325.68), inchToMeter(241.64)), // 14
    new Translation2d(inchToMeter(325.68), inchToMeter(75.39)), // 15
    new Translation2d(inchToMeter(235.73), inchToMeter(-0.15)), // 16
    new Translation2d(inchToMeter(160.39), inchToMeter(130.17)), // 17
    new Translation2d(inchToMeter(144.00), inchToMeter(158.50)), // 18
    new Translation2d(inchToMeter(160.39), inchToMeter(186.83)), // 19
    new Translation2d(inchToMeter(193.10), inchToMeter(186.83)), // 20
    new Translation2d(inchToMeter(209.49), inchToMeter(158.50)), // 21
    new Translation2d(inchToMeter(193.10), inchToMeter(130.17)),// 22
};

//angle of every tag reletive to the field
public static Rotation2d[] TAG_ANGLE = { 
  Rotation2d.kZero, // 0
  Rotation2d.fromDegrees(126), // 1
  Rotation2d.fromDegrees(234), // 2
  Rotation2d.fromDegrees(270), // 3
  Rotation2d.fromDegrees(0), // 4
  Rotation2d.fromDegrees(0), // 5
  Rotation2d.fromDegrees(300), // 6
  Rotation2d.fromDegrees(0), // 7
  Rotation2d.fromDegrees(60), // 8
  Rotation2d.fromDegrees(120), // 9
  Rotation2d.fromDegrees(180), // 10
  Rotation2d.fromDegrees(240), // 11
  Rotation2d.fromDegrees(54), // 12
  Rotation2d.fromDegrees(306), // 13
  Rotation2d.fromDegrees(180), // 14
  Rotation2d.fromDegrees(180), // 15
  Rotation2d.fromDegrees(90), // 16
  Rotation2d.fromDegrees(240), // 17
  Rotation2d.fromDegrees(180), // 18
  Rotation2d.fromDegrees(120), // 19
  Rotation2d.fromDegrees(60), // 20
  Rotation2d.fromDegrees(0), // 21
  Rotation2d.fromDegrees(300),// 22
  };

        /**
         * Array of AprilTag heights from the ground.
         * Each value corresponds to either LOW, MID, or HIGH mounting position.
         * Index corresponds to AprilTag ID (0 is unused).
         */
public static double[] TAG_HEIGHT = { 
  0, // 0
  STATION_TAG_HIGHT, // 1
  STATION_TAG_HIGHT, // 2
  SIDE_TAG_HIGHT, // 3
  BARGE_TAG_HIGHT, // 4
  BARGE_TAG_HIGHT, // 5
  REEF_TAG_HIGHT, // 6
  REEF_TAG_HIGHT, // 7
  REEF_TAG_HIGHT, // 8
  REEF_TAG_HIGHT, // 9
  REEF_TAG_HIGHT, // 10
  REEF_TAG_HIGHT, // 11
  STATION_TAG_HIGHT, // 12
  STATION_TAG_HIGHT, // 13
  BARGE_TAG_HIGHT, // 14
  BARGE_TAG_HIGHT, // 15
  SIDE_TAG_HIGHT, // 16
  REEF_TAG_HIGHT, // 17
  REEF_TAG_HIGHT, // 18
  REEF_TAG_HIGHT, // 19
  REEF_TAG_HIGHT, // 20
  REEF_TAG_HIGHT, // 21
  REEF_TAG_HIGHT,// 22
  };

  public static double inchToMeter(double inch) {
  return inch * 0.0254;
  }


  public static final double REEF_X_OFFSET = 0.50;
  public static final double REEF_LEFT_OFFSET = 0.04;
  public static final double REEF_RIGHT_OFFSET = 0.36;

  public static final Translation2d LeftReefVector = new Translation2d(REEF_X_OFFSET, REEF_LEFT_OFFSET);
  public static final Translation2d RightReefVector = new Translation2d(REEF_X_OFFSET, REEF_RIGHT_OFFSET);
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class elevatorConfig {

    public static final double KG = 0.2;

    public static final TalonConfig LeftMotor = new TalonConfig(30, Canbus.Rio, "Elevator/Left")
        .withBrake(true)
        .withInvert(false)
        .withMeterMotor(15, 0.0364)
        .withPID(15, 0, 0, 0.6, 15, 0.35, KG)
        .withCurrent(35)
        .withRampTime(0.2)
        .withVolts(12);
        
    
    public static final TalonConfig RightMotor = new TalonConfig(31, "Elevator/Right", LeftMotor)
        .withInvert(!LeftMotor.inverted);

    public static final int MagneticLimitSwitchID = 2;
    public static final int LimitSwitchID = 1;
    public static final double CalibreatePowerUp = 0.2;
    public static final double CalibreatePowerDown = 0.05;
  }

  public static final class Arm {
    public static final TalonConfig GripperConfig = new TalonConfig(0, Canbus.CANIvore, "Arm/Gripper")
        .withBrake(true)
        .withInvert(false)
        .withMeterMotor(0.7, 0.0762)
        .withPID(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) 
        .withVolts(12)
        .withCurrent(20)
        .withRampTime(0.2);

    public static final double ARM_CANCODER_OFFSET = 141.8;
    public static final double kG = 0.20;


    public static final TalonConfig ARM_CONFIG = new TalonConfig(16, Canbus.CANIvore, "Arm/Motor")
        .withBrake(true)
        .withInvert(true)
        .withDegreesMotor(108) 
        .withMotionParam(90, 150, 300) 
        .withPID(0.3, 0.0, 0.001, 0.0, 0.0, 0.0, kG) 
        .withVolts(12)
        .withCurrent(30)
        .withRampTime(0.3);

    public static final CancoderConfig ARM_CANCODER = new CancoderConfig(33, Canbus.Rio, "Arm/Cancoder")
        .withOffset(0)
        .withInvert(false);

  }
}