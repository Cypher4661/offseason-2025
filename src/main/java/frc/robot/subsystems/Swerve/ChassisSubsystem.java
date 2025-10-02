package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChassisSubsystem extends SubsystemBase {
    public CommandXboxController Controller;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public AHRS gyro;
    public Field2d field = new Field2d();
    public SwerveModule FL;
    public SwerveModule FR;
    public SwerveModule BR;
    public SwerveModule BL;
    public Pose2d pose = new Pose2d();
    public final SwerveModule[] Modules;
    public boolean PrecisionMode = false;

    public static int N = 0;

    public ChassisSubsystem(){
        super();
        FL  = new SwerveModule(SwerveConstants.ChassisConstants.Config[0]);
        FR  = new SwerveModule(SwerveConstants.ChassisConstants.Config[1]);
        BR = new SwerveModule(SwerveConstants.ChassisConstants.Config[2]);
        BL = new SwerveModule(SwerveConstants.ChassisConstants.Config[3]);
        Modules = new SwerveModule[]{
            FL,
            FR,
            BR,
            BL
        };
        gyro = new AHRS(NavXComType.kMXP_SPI);
      FL.debug(true, this);
    
        kinematics = new SwerveDriveKinematics(new Translation2d[]{
            new Translation2d(SwerveConstants.ChassisConstants.FL_X, SwerveConstants.ChassisConstants.FL_Y),
            new Translation2d(SwerveConstants.ChassisConstants.FR_X, SwerveConstants.ChassisConstants.FR_Y),
            new Translation2d(SwerveConstants.ChassisConstants.BR_X, SwerveConstants.ChassisConstants.BR_Y),
            new Translation2d(SwerveConstants.ChassisConstants.BL_X, SwerveConstants.ChassisConstants.BL_Y)
        });
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getModulePositions(), new Pose2d());
        SmartDashboard.putData("Chassis", this);
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Stop Chassis", new InstantCommand(()-> StopChassis(),this));
        MotorCommands.showRandomPowerCommand("Steer Random Power", -6.0, 6.0, 0.2, this, FL.SteerMotor, FR.SteerMotor, BR.SteerMotor, BL.SteerMotor);
        MotorCommands.showRandomPowerCommand("Drive Random Power", -3, 3, 0.2, this, FL.DriveMotor, FR.DriveMotor, BR.DriveMotor, BL.DriveMotor);
        SmartDashboard.putData("Set 90 Degrees", new RunCommand(()-> setAllModulsTo(90),this));
        SmartDashboard.putData("Set 0 Degrees", new RunCommand(()-> setAllModulsTo(0),this));
        SmartDashboard.putData("Brake", new InstantCommand(()-> setBrake(true)).ignoringDisable(true));
        SmartDashboard.putData("Coast", new InstantCommand(()-> setBrake(false)).ignoringDisable(true));
    
    }    

    public void  setAllModulsTo(double angle){
        FL.setStats(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
        FR.setStats(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
        BR.setStats(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
        BL.setStats(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    }

    public void setBrake(boolean brake) {
        System.out.println("Set brake to " + brake);
        for(SwerveModule module : Modules){
            module.SteerMotor.setNeutralMode(brake);
            module.DriveMotor.setNeutralMode(brake);
        }
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
        return new SwerveModuleState[]{
            FL.getStats(),
            FR.getStats(),
            BR.getStats(),
            BL.getStats()
        };
    }

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            FL.getModulePosition(),
            FR.getModulePosition(),
            BR.getModulePosition(),
            BL.getModulePosition()
        };
    }

    public ChassisSpeeds getChassisSpeedsFieldRel(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getGyroAngle(){
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }
    public Rotation2d getHeading() {
        return pose.getRotation();
    }

   
    public void resetPose(Pose2d pose){
        poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public void setZeroHeading() {
        resetPose(new Pose2d(pose.getTranslation(), Rotation2d.kZero));
    }

    public void setModuleStates(SwerveModuleState[] states){
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
        Translation2d limitedVelocityVector = calculateVelocityWithAccel(wanted_Speeds.vxMetersPerSecond, wanted_Speeds.vyMetersPerSecond);
        ChassisSpeeds limitedVelocities = new ChassisSpeeds(limitedVelocityVector.getX(), limitedVelocityVector.getY(), wanted_Speeds.omegaRadiansPerSecond);
        setVelocities(limitedVelocities);

    }
    public void setVelocities(ChassisSpeeds speeds){
        if(Math.abs(speeds.vxMetersPerSecond) < 0.01 && Math.abs(speeds.vyMetersPerSecond) < 0.01 && Math.abs(speeds.omegaRadiansPerSecond) < 0.01){
            for(SwerveModule m : Modules){
                m.setStop();;    
            }
        } else {
            ChassisSpeeds rspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(rspeeds);
            setModuleStates(states);
        }
    }

    
    public boolean isRed(){
        return RobotContainer.isRed;
    }

    @Override
    public void periodic() {
        Rotation2d gyro = getGyroAngle();
        poseEstimator.update(gyro, getModulePositions());
        pose = poseEstimator.getEstimatedPosition();
        field.setRobotPose(pose);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Gyro",()->getHeading().getDegrees(), null);
    }

    public static Pose2d getTargetForReef(int tagId, boolean left, boolean isBlue) {
        if((isBlue &&  (tagId < 17 || tagId > 22)) || (!isBlue && (tagId < 6 || tagId > 11))) {
            return null;
        }
        Translation2d tagPos = Constants.O_TO_TAG[tagId];
        Rotation2d tagRot = Constants.TAG_ANGLE[tagId];
        Translation2d vec = left ? Constants.LeftReefVector : Constants.RightReefVector;
        return new Pose2d(tagPos.plus(vec.rotateBy(tagRot)), tagRot.plus(Rotation2d.k180deg));
    }

    public static int redReefTag(int blueReefTag) {
        switch (blueReefTag) {
            case 17: return 8;
            case 18: return 7;
            case 19: return 6;
            case 20: return 11;
            case 21: return 10;
            case 22: return 9;
            default: return 0;
        }
    }

}
