package frc.robot;

import frc.Demacia.utils.Motors.TalonConfig;
import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Demacia.utils.Sensors.CancoderConfig;

public final class Constants {

  public static final class Arm {

    public static final int ARM_MOTOR_ID = 3;
    public static final int ARM_CANCODER_ID = 9;

    public static final double ARM_GEAR_RATIO = 1.0 / 100.0;

    public static final boolean ARM_MOTOR_INVERTED = false;
    public static final boolean ARM_CANCODER_INVERTED = false;

    public static final double ARM_CANCODER_OFFSET_RAD = 0.0;

    public static final double VEL_SLOW  = 90.0;
    public static final double VEL_MID   = 150.0;
    public static final double VEL_FAST  = 220.0;

    public static final double kP = 0.03;
    public static final double kI = 0.00;
    public static final double kD = 0.001;
    public static final double kS = 0.00;
    public static final double kV = 0.00;
    public static final double kA = 0.00;
    public static final double kG = 0.60;

    public static final double MAX_VOLTS   = 12.0;
    public static final double MIN_VOLTS   = -12.0;
    public static final double MAX_CURRENT = 30.0;
    public static final double RAMP_TIME_S = 0.30;

    public static final TalonConfig ARM_CONFIG_DEGREES =
        new TalonConfig(ARM_MOTOR_ID, Canbus.Rio, "arm")
          .withBrake(true)
          .withInvert(ARM_MOTOR_INVERTED)
          .withDegreesMotor(ARM_GEAR_RATIO)                 // -> setAngle/getCurrentAngle במעלות
          .withMotionParam(VEL_SLOW, VEL_MID, VEL_FAST)      // deg/s
          .withPID(kP, kI, kD, kS, kV, kA, kG)              // כייל בשטח
          .withVolts(MAX_VOLTS)
          .withCurrent(MAX_CURRENT)
          .withRampTime(RAMP_TIME_S);

    public static final CancoderConfig ARM_CANCODER =
        new CancoderConfig(ARM_CANCODER_ID, Canbus.Rio, "arm/cancoder")
          .withOffset(ARM_CANCODER_OFFSET_RAD)
          .withInvert(ARM_CANCODER_INVERTED);
  }
}
