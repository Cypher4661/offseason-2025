package frc.robot.subsystems.Swerve;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Sensors.Cancoder;

public class SwerveModule implements Sendable{
    protected TalonMotor DriveMotor;
    protected SparkMotor SteerMotor;
    private Cancoder CanCoder;
    public ModuleConfig moduleConfig;

    public boolean debug = false;


    SwerveModule(ModuleConfig config) {
        super();
        this.moduleConfig = config;
        this.DriveMotor = new TalonMotor(config.DriveConfig);
        System.out.println("steer id = " + config.SteerConfig.id);
        this.SteerMotor = new SparkMotor(config.SteerConfig);
        System.out.println("Init cancoder " + config.CancoderConfig.id);
        this.CanCoder = new Cancoder(config.CancoderConfig);
        SteerMotor.setEncoderPosition(getAbsoluteAngle() - config.CanCoderOffset);
        SmartDashboard.putData(config.Name, this);
        
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
        SteerMotor.setPositionVoltage(angleDegrees);
    }
       
    public double getAngle(){
        return SteerMotor.getCurrentPosition();
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
    public double getDrivePosition(){
        return DriveMotor.getCurrentPosition();
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
    public double getSteerVoltage(){
        return SteerMotor.getCurrentVoltage();
    }
    public double getDriveVoltage(){
        return DriveMotor.getCurrentVoltage();
    }
    public SwerveModuleState getStats(){
        return new SwerveModuleState(getDriveVelocity(), getSteerRotation());
    }

    public void setStats(SwerveModuleState state){
        double wantesAngle = state.angle.getDegrees();
        SmartDashboard.putNumber(" DEBUG wanted", wantesAngle);
        double angle = getAbsoluteAngle() - moduleConfig.CanCoderOffset;
        SmartDashboard.putNumber(" DEBUG angle", angle);
        double diff = wantesAngle - angle;
        SmartDashboard.putNumber(" DEBUG dif1", diff);
        double vel = state.speedMetersPerSecond;
        diff = MathUtil.inputModulus(diff,-180, 180);
        SmartDashboard.putNumber(" DEBUG dif2", diff);
        if(diff > 90){
            vel = -vel;
            diff = diff - 180;
        }
        else if(diff < -90){
            vel = -vel;
            diff = diff + 180;
        }
        SmartDashboard.putNumber(" DEBUG dif3", diff);
        setSteerAngle(SteerMotor.getCurrentPosition() + diff);
        SmartDashboard.putNumber(" DEBUG tgt", SteerMotor.getCurrentPosition() + diff);
        setDriveVelocity(vel);  
    }

    public void setStop() {
        SteerMotor.setDuty(0);
        DriveMotor.setDuty(0);
    }

    public SwerveModulePosition gModulePosition(){
        return new SwerveModulePosition(DriveMotor.getCurrentPosition(),Rotation2d.fromDegrees(getSteerAngle()));
    }
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), getSteerRotation());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Abs", this::getAbsoluteAngle, null);
    }

    
}
