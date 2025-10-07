package frc.robot.commands.Autonomous;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class TuneToReef extends Command{
    private Pose2d target;
    private double velocity = SwerveConstants.ChassisConstants.Max_Linear_Speed;
    private ChassisSubsystem chassis;
    private double distanceToTarget = 0;
    private double headingError = 0;
    private VisionSubsystem vision;
    private ElevatorSubsystem elevator;
    private boolean goLeft = false;

    private boolean debug = true;


    private final static double kDistance = SwerveConstants.AutonomousConstants.KDistance;
    private final static double kOmega = SwerveConstants.AutonomousConstants.KOmega;
    private final static double MAX_ERROR = SwerveConstants.AutonomousConstants.Max_Erorr_Riff;
    private final ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
    public TuneToReef(ChassisSubsystem chassis, VisionSubsystem vision, ElevatorSubsystem elevator, boolean goLeft) {
        this.elevator = elevator;
        this.chassis = chassis;
        this.vision = vision;
        this.goLeft = goLeft;
        addRequirements(chassis);
    }

    //double TargetX = SmartDashboard.getNumber("Enter Target X", 0);
    //double TargetY = SmartDashboard.getNumber("Enter Target Y", 0);
    //Rotation2d TargetAngle = new Rotation2d(SmartDashboard.getNumber("Enter Target Angle", 0));


    public Pose2d EnterToTarget(double X, double Y){
        return new Pose2d(X, Y, chassis.getHeading());

    }

    @Override
    public void execute(){
        
        if(vision.isSeeTag()) {
            int id = vision.getTagId();
            if (id < 0 ) id = 0;
            if (id > 22) id =22;
            Translation2d tagPosition = Constants.O_TO_TAG[id];
            Rotation2d tagRotation = Constants.TAG_ANGLE[id];
            double leftorRightdistance = 0.0;
            double distanceFromeReef = 0.0;

            
            


            switch (elevator.getMode()) {
                case L4:
                    distanceFromeReef = goLeft ? SwerveConstants.AutonomousConstants.Left_L4_Reef_X: SwerveConstants.AutonomousConstants.Right_L4_Reef_X;
                    leftorRightdistance = goLeft ? SwerveConstants.AutonomousConstants.Left_Reef_Y_L4 : SwerveConstants.AutonomousConstants.Right_Reef_Y_L4;
                    break;
                case L3:
                    distanceFromeReef = goLeft ? SwerveConstants.AutonomousConstants.Left_L3_Reef_X: SwerveConstants.AutonomousConstants.Right_L3_Reef_X;
                    leftorRightdistance = goLeft ? SwerveConstants.AutonomousConstants.Left_Reef_Y_L3 : SwerveConstants.AutonomousConstants.Right_Reef_Y_L3;
                    break;
                case L2:
                    distanceFromeReef = goLeft ? SwerveConstants.AutonomousConstants.Left_L2_Reef_X: SwerveConstants.AutonomousConstants.Right_L2_Reef_X;
                    leftorRightdistance = goLeft ? SwerveConstants.AutonomousConstants.Left_Reef_Y_L2 : SwerveConstants.AutonomousConstants.Right_Reef_Y_L2;
                    break;
                default:
                    distanceFromeReef = 0.6;
                    leftorRightdistance = goLeft ? SwerveConstants.AutonomousConstants.Left_Reef_Y : SwerveConstants.AutonomousConstants.Right_Reef_Y;
                    break;
            }
            target = new Pose2d(tagPosition.plus(new Translation2d(distanceFromeReef, leftorRightdistance).rotateBy(tagRotation)), 
                tagRotation.plus(Rotation2d.k180deg));
            if(debug) {
                SmartDashboard.putNumber("ToReef Tag angle", tagRotation.getDegrees());
                SmartDashboard.putNumber("ToReef Target id", id);
                SmartDashboard.putNumber("ToReef Target x", target.getX());
                SmartDashboard.putNumber("ToReef Target y", target.getY());
                SmartDashboard.putNumber("ToReef Target angle", target.getRotation().getDegrees());
                SmartDashboard.putBoolean("ToReef see", true);
            }
        } else if(debug) {
            SmartDashboard.putBoolean("ToReef see", false);
        }

        if(target != null) {
            Pose2d currentPosition = chassis.getPose();
            Translation2d toTarget = target.getTranslation().minus(currentPosition.getTranslation());
            distanceToTarget = toTarget.getNorm();
            double Vel = Math.min(velocity, distanceToTarget * kDistance);
            
            headingError = MathUtil.inputModulus(target.getRotation().minus(currentPosition.getRotation()).getRadians(), -Math.PI, Math.PI);
            double omega = headingError*kOmega;
            if(debug) {
                SmartDashboard.putNumber("ToReef dist", distanceToTarget);
                SmartDashboard.putNumber("ToReef Vel", Vel);
                SmartDashboard.putNumber("ToReef omega", omega);
                SmartDashboard.putNumber("ToReef vx", toTarget.getX());
                SmartDashboard.putNumber("ToReef vy", toTarget.getY());
            }
            chassis.setVelocities(new ChassisSpeeds(toTarget.getAngle().getCos() * Vel, toTarget.getAngle().getSin() * Vel, omega));
        } else {
            chassis.setVelocities(new ChassisSpeeds(0,0,0));
            if(debug) {
                SmartDashboard.putNumber("ToReef dist", 0);
                SmartDashboard.putNumber("ToReef Vel", 0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setVelocities(new ChassisSpeeds(0,0,0));
        if(debug) {
            SmartDashboard.putNumber("ToReef end x", chassis.getPose().getX());
            SmartDashboard.putNumber("ToReef end y", chassis.getPose().getY());
            SmartDashboard.putNumber("ToReef end angle", chassis.getPose().getRotation().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        if(debug) SmartDashboard.putBoolean("ToReef end", distanceToTarget < MAX_ERROR && headingError < SwerveConstants.AutonomousConstants.Max_Heading_Erorr_Riff);
        return distanceToTarget < MAX_ERROR && headingError < SwerveConstants.AutonomousConstants.Max_Heading_Erorr_Riff;
    }
}

