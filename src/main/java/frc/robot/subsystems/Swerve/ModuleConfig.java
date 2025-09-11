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
    public boolean IsSteerInverted = true;
    public boolean IsDriveInverted = false;

    public ModuleConfig(int moduleID, int steerID, int driveID, int canCoderID, String name, 
            double diameter, double canCoderOffset) {
        this.ModuleID = moduleID;
        this.SteerID = steerID;
        this.DriveID = driveID;
        this.CanCoderID = canCoderID;
        this.Name = name;
        this.Diameter = diameter;
        this.CanCoderOffset = canCoderOffset;
        

        SteerConfig = new SparkConfig(steerID, name + "/Steer")
            .withBrake(true)
            .withInvert(IsSteerInverted)
            .withDegreesMotor(GearRatioSteer)
            .withCurrent(20)
            .withVolts(8)
            .withPID(0.9, 0.001, 0, 0.3, 0.0072425, 0.00067, 0)
            .withRampTime(0.2)
            .withMotionParam(720, 1200, 2000)
            .withMaxPositionError(0.1);

        DriveConfig = new TalonConfig(driveID, Canbus.Rio, name + "/Drive")
            .withBrake(true)
            .withInvert(IsDriveInverted)
            .withMeterMotor(GearRatioDrive, diameter)
            .withCurrent(20)
            .withVolts(6)
            .withPID(4, 0, 0, 0.27, 2.37251, 0.1251425, 0)
            .withRampTime(0.2)
            .withMotionParam(720, 1200, 2000);

        CancoderConfig = new CancoderConfig(canCoderID, Canbus.Rio, name + "/Cancoder");
            
    }
}
