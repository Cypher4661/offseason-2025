package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.ChassisSubsystem;

public class GoToCommand extends Command {
    private Pose2d target;
    private double velocity;
    private ChassisSubsystem chassis;
    private double distanceToTarget = 0;

    private final static double kDistance = 2;
    private final static double kOmega = 2;
    private final static double MAX_ERROR = 0.05;
    private final ProfiledPIDController pid;

    public GoToCommand(Pose2d target, double velocity, ChassisSubsystem chassis) {
        this.target  = target;
        this.chassis = chassis;
        this.velocity = velocity;
        this.pid = new ProfiledPIDController(kDistance, 0, 0, new Constraints(velocity, velocity * 2));
        addRequirements(chassis);
    }

    double angleError = 0;
    @Override
    public void execute() {
        Pose2d currentPosition = chassis.getPose();
        Translation2d toTarget = target.getTranslation().minus(currentPosition.getTranslation());
        
        distanceToTarget = toTarget.getNorm();
        angleError = target.getRotation().minus(currentPosition.getRotation()).getRadians();
        double v = Math.min(velocity, pid.calculate(-distanceToTarget, 0));
        double omega = angleError*kOmega;
        // System.out.println("Distance: " + distanceToTarget + " Angle Error: " + Math.toDegrees(angleError) + " v: " + v + " omega: " + omega);
        ChassisSpeeds s;
        if(distanceToTarget < MAX_ERROR) s = new ChassisSpeeds(0, 0, omega);
        
        else chassis.setVelocities(new ChassisSpeeds(v * toTarget.getAngle().getCos(), v * toTarget.getAngle().getSin(), omega));
        }

    @Override
    public boolean isFinished() {
        return distanceToTarget < MAX_ERROR && Math.abs(Math.toDegrees(angleError)) < 3;
    }


}
