package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.Demacia.utils.Motors.TalonConfig;
import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;



public class ProTalonConfig {
    public final int id;
    public final String name;
    public final double ratio;
    public final boolean inverted;
    public final double diameter;
    public final double rampTime;
    public final frc.Demacia.utils.Motors.TalonConfig ProTalonConfig;
    

    public ProTalonConfig(int id, String name, double ratio, boolean inverted, double diameter, double rampTime) {
        this.id = id;
        this.name = name;
        this.ratio = ratio;
        this.inverted = inverted;
        this.diameter = diameter;
        this.rampTime = rampTime;
        ProTalonConfig = new TalonConfig(id, Canbus.Rio, name)
        .withBrake(true)
        .withInvert(inverted)
        //.withRadiansMotor(ratio)
        .withMeterMotor(ratio, diameter) // diameter in meters, wheel radius in meters
        //.withDegreesMotor(ratio)
        .withMotionParam(0.2, 6, 10) // m/s
        .withPID(4, 0, 0, 0.12, 60, 0.1, 0)
        .withRampTime(rampTime)
        .withVolts(12)
        .withCurrent(20);
    }

    
}
