package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Sensors.Cancoder;

public class SwerveModule implements Sendable {
    private MotorInterface steer;
    private MotorInterface drive;
    private Cancoder absEncoder;
    protected Constants.ModuleConfig config;
    protected SwerveModuleState state = new SwerveModuleState();
    protected SwerveModulePosition position = new SwerveModulePosition();


    SwerveModule(Constants.ModuleConfig config) {
        this.config = config;
        steer = new SparkMotor(config.steerConfig);
        drive = new TalonMotor(config.driveConfig);
        absEncoder = new Cancoder(config.cancoderConfig);
        setSteerOffset();
        refresh();
        SmartDashboard.putData(config.name, this);
    }

    public void setSteerOffset() {
        steer.setEncoderPosition(getAbsEncoder()-config.cancoderOffset);        
    }

    public double getAbsEncoder() {
        return absEncoder.getCurrentAbsPosition();
    }

    public void refresh() {
        state.angle.setDegrees(steer.getCurrentPosition());
        state.speedMetersPerSecond = drive.getCurrentVelocity();
        position.angle.set(state.angle.getRadians());
        position.distanceMeters = drive.getCurrentPosition() + steer.getCurrentPosition() * Constants.STEER_TO_DISTANCE_RATIO;
    }

    public void setState(SwerveModuleState state) {
        state.optimize(this.state.angle);
        steer.setAngle(state.angle.getDegrees());
        drive.setVelocity(state.speedMetersPerSecond);
    }

    public void setSteerPower(double power) {
        steer.setDuty(power);
    }
    public void setDrivePower(double power) {
        drive.setDuty(power);
    }
    public void setSteerAngle(double angle) {
        steer.setAngle(angle);
    }
    public void setDriveVelocity(double velocity) {
        drive.setVelocity(velocity);
    }

    public void showSysidCommads(Subsystem subsystem) {
        steer.showSysidCommands(subsystem);
        drive.showSysidCommands(subsystem);
    }

    protected MotorInterface steerMotor() {
        return steer;
    }
    protected MotorInterface driveMotor() {
        return drive;
    }

    public void setBrake() {
        steer.setNeutralMode(true);
        drive.setNeutralMode(true);
    }
    public void setCoast() {
        steer.setNeutralMode(false);
        drive.setNeutralMode(false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("AbsEncoder",this::getAbsEncoder, null);
    }
}
