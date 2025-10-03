package frc.robot.commands.Autonomous;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.Swerve.SwerveConstants;
//import frc.robot.subsystems.VisionSubsystem;\
import frc.robot.Constants;

public class TuneToRiff extends Command{
    private Pose2d target;
    private double velocity;
    private ChassisSubsystem chassis;
    private double distanceToTarget = 0;
    //private VisionSubsystem vision;

    private final static double kDistance = SwerveConstants.AutonomousConstants.KDistance;
    private final static double kOmega = SwerveConstants.AutonomousConstants.KOmega;
    private final static double MAX_ERROR = SwerveConstants.AutonomousConstants.Max_Erorr_Riff;

    public TuneToRiff(Pose2d target, double velocity, ChassisSubsystem chassis) {
        this.target  = target;
        this.chassis = chassis;
        this.velocity = velocity;
        addRequirements(chassis);
    }

    public Pose2d EnterToTarget(double X, double Y){
        return new Pose2d(X, Y, chassis.getHeading());

    }

    @Override
    public void execute(){
        //int aprilID = vision.getTagId();
        //double aprilPosition = Constants.Translation2d(O_T0_TAG = aprilID);
        //double aprilOmega = Constants.Rotation2d(TAG_ANGLE = aprilID);
        //target = new Pose2d(aprilPosition, aprilOmega);




        Pose2d currentPosition = chassis.getPose();
        Translation2d toTarget = target.getTranslation().minus(currentPosition.getTranslation());
        distanceToTarget = toTarget.getNorm();
        double Vel = Math.min(velocity, distanceToTarget * kDistance);
        toTarget.times(Vel/distanceToTarget);
        double omega = toTarget.getAngle().minus(currentPosition.getRotation()).getRadians()*kOmega;
        chassis.setVelocities(new ChassisSpeeds(toTarget.getX(), toTarget.getY(), omega));
    }

    @Override
    public boolean isFinished() {
        return distanceToTarget < MAX_ERROR;
    }
}
