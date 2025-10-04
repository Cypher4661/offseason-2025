package frc.robot.commands.Autonomous;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.Swerve.SwerveConstants;

public class TuneToRiff extends Command{
    private Pose2d target;
    private double velocity = SwerveConstants.ChassisConstants.Max_Linear_Speed;
    private ChassisSubsystem chassis;
    private double distanceToTarget = 0;
    private VisionSubsystem vision;

    private final static double kDistance = SwerveConstants.AutonomousConstants.KDistance;
    private final static double kOmega = SwerveConstants.AutonomousConstants.KOmega;
    private final static double MAX_ERROR = SwerveConstants.AutonomousConstants.Max_Erorr_Riff;

    public TuneToRiff(Pose2d target, ChassisSubsystem chassis, VisionSubsystem vision) {
        this.target  = target;
        this.chassis = chassis;
        this.vision = vision;
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
            Translation2d tagPosition = Constants.O_TO_TAG[id];
            Rotation2d tagRotation = Constants.TAG_ANGLE[id];
            // need code for left/right and level
            double xOffset = 0.52; // 52 cm from reef
            double yOffset = -0.06; // 6 cm left of reef
            target = new Pose2d(tagPosition.plus(new Translation2d(xOffset, yOffset).rotateBy(tagRotation)), 
                tagRotation.plus(Rotation2d.k180deg));
        }

        if(target != null) {
            Pose2d currentPosition = chassis.getPose();
            Translation2d toTarget = target.getTranslation().minus(currentPosition.getTranslation());
            distanceToTarget = toTarget.getNorm();
            double Vel = Math.min(velocity, distanceToTarget * kDistance);
            toTarget.times(Vel/distanceToTarget);
            double omega = toTarget.getAngle().minus(currentPosition.getRotation()).getRadians()*kOmega;
            chassis.setVelocities(new ChassisSpeeds(toTarget.getX(), toTarget.getY(), omega));
        } else {
            chassis.setVelocities(new ChassisSpeeds(0,0,0));
        }
    }

    @Override
    public boolean isFinished() {
        return distanceToTarget < MAX_ERROR;
    }
}
