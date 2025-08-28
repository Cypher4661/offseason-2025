package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.ForkJoinTask;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ChassisSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public Pigeon2 gyro;
    public Field2d field = new Field2d();
    public SwerveModule FL;
    public SwerveModule FR;
    public SwerveModule BR;
    public SwerveModule BL;
    public final SwerveModule[] Modules = {
        FL = new SwerveModule(SwerveConstants.ChassisConstants.Config[0]),
        FR = new SwerveModule(SwerveConstants.ChassisConstants.Config[1]),
        BR = new SwerveModule(SwerveConstants.ChassisConstants.Config[2]),
        BL = new SwerveModule(SwerveConstants.ChassisConstants.Config[3])
    };

    public ChassisSubsystem(){
        super();
        gyro = new Pigeon2(SwerveConstants.ChassisConstants.GyroID, "rio");
        kinematics = new SwerveDriveKinematics(new Translation2d[]{
            new Translation2d(SwerveConstants.ChassisConstants.FL_X, SwerveConstants.ChassisConstants.FL_Y),
            new Translation2d(SwerveConstants.ChassisConstants.FR_X, SwerveConstants.ChassisConstants.FR_Y),
            new Translation2d(SwerveConstants.ChassisConstants.BR_X, SwerveConstants.ChassisConstants.BR_Y),
            new Translation2d(SwerveConstants.ChassisConstants.BL_X, SwerveConstants.ChassisConstants.BL_Y)
        });
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getModulePositions(), new Pose2d());
    }

    public void StopChassis(){
        FL.stop();
        FR.stop();
        BR.stop();
        BL.stop();
    }

    public void setNeutralMode(boolean isBrake){
        for(SwerveModule module : Modules){
            module.setNeutralMode(isBrake);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        return kinematics.toSwerveModuleStates(getChassisSpeedsFieldRel());
    }

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            new SwerveModulePosition(FL.getDrivePosition(), FL.getStats().angle),
            new SwerveModulePosition(FR.getDrivePosition(), FR.getStats().angle),
            new SwerveModulePosition(BR.getDrivePosition(), BR.getStats().angle),
            new SwerveModulePosition(BL.getDrivePosition(), BL.getStats().angle)
        };
    }

    public ChassisSpeeds getChassisSpeedsFieldRel(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getGyroAngle(){
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void setYaw(Rotation2d yaw){
        gyro.setYaw(yaw.getDegrees());
    }

    public void resetPose(Pose2d pose){
        poseEstimator.resetPose(pose);
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public void setModuleStats(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.ChassisConstants.Max_Linear_Speed);
        FL.setStats(states[0]);
        FR.setStats(states[1]);
        BR.setStats(states[2]);
        BL.setStats(states[3]);
    }

    public Translation2d calculateVelocityWithAccel(double wanted_Vx, double wanted_Vy){
        ChassisSpeeds currentSpeeds = getChassisSpeedsFieldRel();
        double current_Vx = currentSpeeds.vxMetersPerSecond;
        double current_Vy = currentSpeeds.vyMetersPerSecond;

        double X_Accel = (wanted_Vx - current_Vx) / 0.02;
        double Y_Accel = (wanted_Vy - current_Vy) / 0.02;

        if(Math.abs(X_Accel)> SwerveConstants.ChassisConstants.Max_Linear_Accel){
            X_Accel = Math.signum(X_Accel) * SwerveConstants.ChassisConstants.Max_Linear_Accel;
        }
        if(Math.abs(Y_Accel)> SwerveConstants.ChassisConstants.Max_Linear_Accel){
            Y_Accel = Math.signum(Y_Accel) * SwerveConstants.ChassisConstants.Max_Linear_Accel;
        }
        return new Translation2d(current_Vx + X_Accel * 0.02, current_Vy + Y_Accel * 0.02);
    }

    public void setVelocityWithAccel(ChassisSpeeds wanted_Speeds){
        ChassisSpeeds currentSpeeds = getChassisSpeedsFieldRel();
        Translation2d limitedVelocityVector = calculateVelocityWithAccel(wanted_Speeds.vxMetersPerSecond, wanted_Speeds.vyMetersPerSecond);
        ChassisSpeeds limitedVelocities = new ChassisSpeeds(limitedVelocityVector.getX(), limitedVelocityVector.getY(), wanted_Speeds.omegaRadiansPerSecond);
        Translation2d lastWantedSpeeds = limitedVelocityVector;
        setVelocities(limitedVelocities);
        
    }

    public void setVelocities(ChassisSpeeds speeds){
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        setModuleStats(states);
    }

    @Override
    public void periodic() {
        Rotation2d gyro = getGyroAngle();
        poseEstimator.update(gyro, getModulePositions());
    }

}
