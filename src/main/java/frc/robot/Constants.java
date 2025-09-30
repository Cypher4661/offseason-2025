// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Demacia.utils.Motors.TalonConfig;

import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Demacia.utils.Sensors.CancoderConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final class elevatorConfig {

    public static final double KG = 0.265;

    public static final TalonConfig LeftMotor = new TalonConfig(30, Canbus.Rio, "Elevator/Left")
        .withBrake(true)
        .withInvert(false)
        .withMeterMotor(15, 0.047)
        .withPID(15, 0, 0, 0.25, 11.5, 0.145, KG)
        .withCurrent(30)
        .withRampTime(0.5)
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

    public static final double ARM_CANCODER_OFFSET = 288;
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
