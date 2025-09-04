package frc.robot.subsystems.Swerve;

import frc.Demacia.utils.Motors.SparkConfig;
import frc.Demacia.utils.Motors.TalonConfig;
import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Demacia.utils.Sensors.CancoderConfig;

public class ModuleConfig {
    public int ModuleID;
    public SparkConfig SteerConfig;
    public TalonConfig DriveConfig;
    public CancoderConfig CancoderConfig;
    public int CanCoderID;
    public String Name;
    public int SteerID;
    public int DriveID;
    public double GearRatioSteer = 150.0 / 7.0;
    public double GearRatioDrive = 6.75;
    public double Diameter;
    public double CanCoderOffset;

    public ModuleConfig(int moduleID, int steerID, int driveID, int canCoderID, String name, double diameter, double canCoderOffset) {
        this.ModuleID = moduleID;
        this.SteerID = steerID;
        this.DriveID = driveID;
        this.CanCoderID = canCoderID;
        this.Name = name;
        this.Diameter = diameter;
        this.CanCoderOffset = canCoderOffset;

        CancoderConfig = new CancoderConfig(canCoderID, Canbus.Rio, name + "/Cancoder");

        SteerConfig = new SparkConfig(steerID, name + "/Steer")
            .withBrake(true)
            .withInvert(false)
            .withDegreesMotor(GearRatioSteer)
            .withCurrent(20)
            .withVolts(8)
            .withPID(0, 0, 0, 0, 0, 0, 0)
            .withRampTime(0.2)
            .withMotionParam(720, 1200, 2000)
            .withMaxPositionError(0.5);

        DriveConfig = new TalonConfig(driveID, Canbus.Rio, name + "/Drive")
            .withBrake(true)
            .withInvert(false)
            .withMeterMotor(GearRatioDrive, diameter)
            .withCurrent(20)
            .withVolts(12)
            .withPID(0, 0, 0, 0, 0, 0, 0)
            .withRampTime(0.2)
            .withMotionParam(720, 1200, 2000);
            
    }
}
