// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.Demacia.utils.Motors.TalonConfig;

import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public  final class Constants {

    public  static final class elevatorConfig{
        public static final double kg = 0.5;

            public static final TalonConfig LeftMotor = new TalonConfig(7, Canbus.Rio, "ElevatorLeft")
            .withBrake(true)
            .withInvert(false)
            .withMeterMotor(60, 2*0.045)
            .withPID(4, 0.1, 0.01, 0, 0, 0, kg)
            .withCurrent(30)
            .withRampTime(0.5)
            .withVolts(12);
    public static final TalonConfig RightMotor = new TalonConfig(2, "ElevatorRight", LeftMotor)
            .withInvert(true);

    public static final int MagneticLimitSwitchID = 0;
    public static final int LimitSwitchID = 1;

            
    }

    }




  





