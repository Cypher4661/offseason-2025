package frc.robot.subsystems.Swerve;


import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Demacia.utils.DriverUtils;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.robot.RobotContainer;
import frc.Demacia.utils.DriverUtils.JoystickSide;

public class SwerveSubsystem extends SubsystemBase {

    SwerveModule[] modules;
    SwerveDrivePoseEstimator poseEstimator;
    Field2d robotField;
    SwerveDriveKinematics kinematics;
    Pigeon2 gyro;
    CommandXboxController controller;

    // per module data
    MotorInterface[] steerMotors;
    MotorInterface[] driveMotors;
    SwerveModulePosition[] modulePositions;
    SwerveModuleState[] moduleStates;

    // data that is updated per cycle
    StatusSignal<Angle> gyroSignal;
    Pose2d pose;
    ChassisSpeeds currentChassisSpeeds;
    ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
    Rotation2d gyroRotation = new Rotation2d();

    public SwerveSubsystem(CommandXboxController controller) {
        super();
        this.controller = controller;
        // create the array of modules data
        modules = new SwerveModule[Constants.CONFIGS.length];
        Translation2d[] modulePositionOnRobot = new Translation2d[modules.length];
        modulePositions = new SwerveModulePosition[modules.length];
        moduleStates = new SwerveModuleState[modules.length];
        steerMotors = new MotorInterface[modules.length];
        driveMotors = new MotorInterface[modules.length];
        // fill the per module data
        for(int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(Constants.CONFIGS[i]);
            modulePositionOnRobot[i] = modules[i].config.positionRelativeToRobotCenter;
            moduleStates[i] = modules[i].state;
            modulePositions[i] = modules[i].position;
            steerMotors[i] = modules[i].steerMotor();
            driveMotors[i] = modules[i].driveMotor();
        }
        // create the remaining data
        kinematics = new SwerveDriveKinematics(modulePositionOnRobot);
        gyro = new Pigeon2(Constants.GYRO_ID, Constants.GYRO_CANBUS.canbus);
        gyroSignal = gyro.getYaw();
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), modulePositions,new Pose2d());
        pose = poseEstimator.getEstimatedPosition();
        robotField = new Field2d();

        // add to elastic
        SmartDashboard.putData("Drive", this);
        SmartDashboard.putData("Robot Position", robotField);
        SmartDashboard.putData("Set Drive Brake", new InstantCommand(()-> {for(SwerveModule m : modules) m.setBrake();}).ignoringDisable(true));
        SmartDashboard.putData("Set Drive Coast", new InstantCommand(()-> {for(SwerveModule m : modules) m.setCoast();}).ignoringDisable(true));
        SmartDashboard.putData("Reset Heading", new InstantCommand(this::setFieldHeading, (SubsystemBase)null).ignoringDisable(true));

        // add default command and driver option to reset the heading
        controller.start().onTrue(new InstantCommand(this::setFieldHeading, (SubsystemBase)null).ignoringDisable(true));
        setDefaultCommand(new RunCommand(this::drive, this));

        // show the base commands - only for Sysid and parameters settings
        showBaseCommands();
    }

    private void showBaseCommands() {
        MotorCommands.showRandomPowerCommand("Steers Random Power", -0.6, 0.6, 0.3, this, steerMotors);
        MotorCommands.showRandomPowerCommand("Drives Random Power", -0.9, 0.9, 0.2, this, driveMotors);
        MotorCommands.showSlowPowerCommand("Steers Slow Power", 0.05, 0.01, 1, this, steerMotors);
        MotorCommands.showSlowPowerCommand("Drives Slow Power", 0.03, 0.01, 1, this, driveMotors);
        MotorCommands.showAngleCommand("Set Steer Angle",this, steerMotors);
        MotorCommands.showVelocityCommand("Set Drive Velocity",this, driveMotors);
        for(SwerveModule m : modules) {
            m.showSysidCommads(this);
        }

    }

    /*
     * Default drive function
     */
    private void drive() {
        targetChassisSpeeds.vxMetersPerSecond = DriverUtils.getJSvalue(controller, JoystickSide.RightY) * Constants.MAX_SPEED;
        targetChassisSpeeds.vyMetersPerSecond = -DriverUtils.getJSvalue(controller, JoystickSide.RightX) * Constants.MAX_SPEED;
        targetChassisSpeeds.omegaRadiansPerSecond = DriverUtils.getTriggerValue(controller) * Constants.MAX_OMEGA;
        setSpeeds(targetChassisSpeeds);
    }

    /*
     * Set the current heading as the filed 0 heading - direction to Red 
     */
    public void setFieldHeading() {
        resetPose(pose.getTranslation(), Rotation2d.kZero);
    }

    public Rotation2d getGyroRotation() {
        return gyroRotation;
    }

    public double getGyroHeading() {
        return gyroRotation.getDegrees();
    }

    public Rotation2d getHeadingRotation() {
        return pose.getRotation();
    }

    public double getHeading() {
        return getHeadingRotation().getDegrees();
    }

    public void resetPose(Translation2d translation2d, Rotation2d rotation2d) {
        poseEstimator.resetPose(new Pose2d(translation2d, rotation2d));
    }


    /*
     * Set the modules state to reach the required ChassisSpeeds
     * Chassis speed in Field Oriented
     */
    public void setSpeeds(ChassisSpeeds speeds) {
        // convert to Robot Oriented
        ChassisSpeeds robotRelativSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeadingRotation());
        // limit speed change base on max acceleration
        limitSpeeds(robotRelativSpeeds);
        // get the required module states
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelativSpeeds);
        // limit the maximum velocity of all mdules
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SPEED);
        // set the module states
        for(int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    /*
     * Limit the maximum velocity change in a cycle - based on maximum Acceleration
     */
    private void limitSpeeds(ChassisSpeeds speeds) {
        // limit robot relative speeds to account for MAX accelration
        double currentX = currentChassisSpeeds.vxMetersPerSecond;
        double currentY = currentChassisSpeeds.vyMetersPerSecond;
        double deltaX = Constants.MAX_X_ACCELERATION * RobotContainer.CYCLE_TIME;
        double deltaY = Constants.MAX_Y_ACCELERATION * RobotContainer.CYCLE_TIME;
        double xClamped = MathUtil.clamp(speeds.vxMetersPerSecond, currentX - deltaX, currentX + deltaX);
        if(xClamped != speeds.vxMetersPerSecond) {
            speeds.vyMetersPerSecond *= xClamped / speeds.vxMetersPerSecond;
            speeds.vxMetersPerSecond = xClamped;
        }
        double yClamped = MathUtil.clamp(speeds.vyMetersPerSecond, currentY - deltaY, currentY + deltaY);
        if(yClamped != speeds.vyMetersPerSecond) {
            speeds.vxMetersPerSecond *= yClamped / speeds.vyMetersPerSecond;
            speeds.vyMetersPerSecond = yClamped;
        }
    }

    /*
     * periodic
     * Update all data once per cycle - gyro, module state and position, chessis speed
     * Update the pose estimation
     * update the current pose and robot position on field
     */
    @Override
    public void periodic() {
        super.periodic();
        gyroSignal.refresh();
        gyroRotation.set(gyroSignal.getValue().in(Radians));
        for(SwerveModule m : modules) {
            m.refresh();
        }
        currentChassisSpeeds = kinematics.toChassisSpeeds(moduleStates);
        poseEstimator.update(getGyroRotation(), modulePositions);
        pose = poseEstimator.getEstimatedPosition();
        robotField.setRobotPose(pose);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Heading", this::getHeading, null);
        builder.addDoubleProperty("Vx", ()->currentChassisSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Vy", ()->currentChassisSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Omega Rad Per Sec", ()->currentChassisSpeeds.omegaRadiansPerSecond, null);
    }

}
