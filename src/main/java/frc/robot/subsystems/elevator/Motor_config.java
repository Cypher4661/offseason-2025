package frc.robot.subsystems.elevator;
import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;


public class Motor_config {
    public final int ID;
    public final String name;
    public final double ratio;
    public final boolean inverted;
    public final double diameter;
    public final double rampTime;
    public final double maxVelocity;
    public final double maxAcceleration;
    public final double maxJerk;
    public final double p;
    public final double i;
    public final double d;
    public final double s;
    public final double v;
    public final double a;
    public final double g;
    public final double maxVolts;
    public final double maxCurrent;

    public final frc.Demacia.utils.Motors.TalonConfig TalonConfig;

    public Motor_config (int ID, String name, double ratio, boolean inverted, double diameter, double rampTime, double maxVelocity, double maxAcceleration, double maxJerk, double p, double i, double d, double s ,double v, double a, double g, double maxVolts, double maxCurrent) {
        this.ID = ID;
        this.name = name;
        this.ratio = ratio;
        this.inverted = inverted;
        this.diameter = diameter;
        this.rampTime = rampTime;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        this.p = p;
        this.i = i;
        this.d = d;
        this.s = s;
        this.v = v;
        this.a = a;
        this.g = g;
        this.maxVolts = maxVolts;
        this.maxCurrent = maxCurrent;
        
        TalonConfig = new frc.Demacia.utils.Motors.TalonConfig(ID, Canbus.Rio, name)
        .withBrake(true)
        .withInvert(inverted)
        .withMeterMotor(ratio, diameter) // diameter in meters, wheel radius in meters
        .withMotionParam(maxVelocity, maxAcceleration, maxJerk) // m/s
        .withPID(p, i, d)
        .withRampTime(rampTime)
        .withVolts(maxVolts)
        .withCurrent(maxVolts);

    }

    
}
