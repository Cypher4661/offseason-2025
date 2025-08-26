package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Sensors.Cancoder;
import frc.robot.subsystems.Swerve.ModuleConfig;

public class ModuleSubsystem extends SubsystemBase{
    private TalonMotor DriveMotor;
    private SparkMotor SteerMotor;
    private Cancoder CanCoder;
    private ModuleConfig ModuleConfig;

    public ModuleSubsystem(ModuleConfig config) {
        super();
        this.DriveMotor = new TalonMotor(config.DriveConfig);
        this.SteerMotor = new SparkMotor(config.SteerConfig);
        this.CanCoder = new Cancoder(config.CancoderConfig);
        
        SteerMotor.setAngle(getAbsoluteAngle() - config.CanCoderOffset);
    }

    public void setNeutralMode(boolean isBrake){
        DriveMotor.setNeutralMode(isBrake);
        SteerMotor.setNeutralMode(isBrake);
    }

    public void stop(){
        DriveMotor.set(0);
        SteerMotor.set(0);
    }
    public void setSteerPower(double power){
        SteerMotor.set(power);
    }
    public void setDrivePower(double power){
        DriveMotor.set(power);
    }
    public void setSteerVelocity(double velocityRadPerSec){
        SteerMotor.setVelocity(velocityRadPerSec);
    }
    public void setDriveVelocity(double velocityMeterPerSec){
        DriveMotor.setVelocity(velocityMeterPerSec);
    }
    public void setSteerAngle(double angleDegrees){
        SteerMotor.setAngle(angleDegrees);
    }
    public double getAngle(){
        return SteerMotor.getCurrentPosition() / ModuleConfig.GearRatioSteer * 360.0;
    }
    public double getAbsoluteAngle(){
        return CanCoder.getCurrentAbsPosition();
    }
    public double getSteerAngle(){
        return getAngle();
    }
    public Rotation2d getSteerRotation(){
        return new Rotation2d(getSteerAngle());
    }
    public double getSteerVelocity(){
        return SteerMotor.getCurrentVelocity();
    }
    public double getDriveVelocity(){
        return DriveMotor.getCurrentVelocity();
    }
    public double getSteerAcceleration(){
        return SteerMotor.getCurrentAcceleration();
    }
    public double getDriveAcceleration(){
        return DriveMotor.getCurrentAcceleration();
    }

    public void setStats(SwerveModuleState state){
        double wantesAngle = state.angle.getDegrees();
        double diff = wantesAngle - getAngle();
        double vel = state.speedMetersPerSecond;
        diff = MathUtil.angleModulus(diff);
        if(diff > 0.5 * Math.PI){
            vel = -vel;
            diff = diff - Math.PI;
        }
        else if(diff < -0.5 * Math.PI){
            vel = -vel;
            diff = diff + Math.PI;
        }
        setSteerAngle(getAngle() + diff);
        setDriveVelocity(vel);  
    }

    public SwerveModulePosition gModulePosition(){
        return new SwerveModulePosition(DriveMotor.getCurrentPosition(),Rotation2d.fromDegrees(getSteerAngle()));
    }
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), getSteerRotation());
    }

    
}
