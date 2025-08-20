package frc.robot;

import frc.Demacia.utils.Motors.SparkConfig;

public class ProSparkConfig {
    public final int sparkId;
    public final String name;
    public final double SparkGearRatio;
    public final boolean isSparkInverted;
    public final double diameter;
    public final double SparkRampTime;
    public final frc.Demacia.utils.Motors.SparkConfig ProSparkConfig;
    

    public ProSparkConfig(String name,int sparkId,
    boolean isSparkInverted, double SparkGearRatio, double SparkRampTime , double diameter 
    ,double SparkMaxVelocity , double SparkMaxAcceleration ,double  SparkMaxJerk, 
    double SparkMaxVoltage , double SparkMaxCurent) 
    {
        this.sparkId = sparkId;
        this.name = name;
        this.SparkGearRatio = SparkGearRatio;
        this.isSparkInverted = isSparkInverted;
        this.diameter = diameter;
        this.SparkRampTime = SparkRampTime;

        ProSparkConfig = new SparkConfig(sparkId ,name )
        .withInvert(isSparkInverted)
        .withMeterMotor(SparkGearRatio, diameter) 
        .withMotionParam(SparkMaxVelocity, SparkMaxAcceleration, SparkMaxJerk) 
        .withRampTime(SparkRampTime)
        .withVolts(SparkMaxVoltage)
        .withPID(0.001, 0, 0, 1.0 / 150, 2.0 / 9, 0, 0)
        .withCurrent(SparkMaxCurent);
        


    }
}
