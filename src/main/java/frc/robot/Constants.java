// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private static double inchToMeter(double inch) {
    return inch * 0.0254;
  }

  public static Translation2d[] O_TO_TAG = { null, // 0
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

public static Rotation2d[] TAG_ANGLE = { null, // 0
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

    public static final double ARM_CANCODER_OFFSET = 157.16;
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