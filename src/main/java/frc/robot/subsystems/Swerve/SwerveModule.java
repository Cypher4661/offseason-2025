package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Sensors.Cancoder;
import frc.robot.subsystems.Swerve.SwerveConstants.ModuleConstants;

public class SwerveModule extends SubsystemBase{
    private TalonMotor DriveMotor;
    private SparkMotor SteerMotor;
    private Cancoder CanCoder;
    public ModuleConfig moduleConfig;

    private SimpleMotorFeedforward SteerFF = new SimpleMotorFeedforward(ModuleConstants.Steer_KS, ModuleConstants.Steer_KV, ModuleConstants.Steer_KA);
    private SimpleMotorFeedforward DriveFF = new SimpleMotorFeedforward(ModuleConstants.Drive_KS, ModuleConstants.Drive_KV, ModuleConstants.Drive_KA);
    private PIDController SteerPID = new PIDController(ModuleConstants.Steer_KP, ModuleConstants.Steer_KI, ModuleConstants.Steer_KD);
    private PIDController DrivePID = new PIDController(ModuleConstants.Drive_KP, ModuleConstants.Drive_KI, ModuleConstants.Drive_KD);

    SwerveModule(ModuleConfig config) {
        super();
        this.DriveMotor = new TalonMotor(config.DriveConfig);
        System.out.println("steer id = " + config.SteerConfig.id);
        this.SteerMotor = new SparkMotor(config.SteerConfig);
        System.out.println("Init cancoder " + config.CancoderConfig.id);
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
        double ff = SteerFF.calculate(velocityRadPerSec);
        double pid = SteerPID.calculate(getSteerVelocity(), velocityRadPerSec);
        setSteerPower(pid + ff);;
    }
    public void setDriveVelocity(double velocityMeterPerSec){
        double ff = DriveFF.calculate(velocityMeterPerSec);
        double pid = DrivePID.calculate(getDriveVelocity(), velocityMeterPerSec);
        setDrivePower(pid + ff);;
    }
    public void setSteerAngle(double angleDegrees){
        SteerMotor.setAngle(angleDegrees);
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
