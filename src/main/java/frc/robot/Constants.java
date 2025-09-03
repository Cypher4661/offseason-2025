
package frc.robot;


import frc.Demacia.utils.Motors.SparkConfig;
import frc.Demacia.utils.Motors.TalonConfig;
import frc.robot.ElevatorMotorConfig;
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

    public static final class ElevatorSubsystem {
            public static final ElevatorMotorConfig talon = new ElevatorMotorConfig(1, "elemotor", 1.0/100.0, false, 0.0762, 0);


    }

  
}




