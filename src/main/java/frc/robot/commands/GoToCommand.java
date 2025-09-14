package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.ChassisSubsystem;

public class GoToCommand extends Command {
    private Pose2d target;
    private double velocity;
    private ChassisSubsystem chassis;
    private double distanceToTarget = 0;

    private final static double kDistance = 2;
    private final static double kOmega = 1;
    private final static double MAX_ERROR = 0.05;

    public GoToCommand(Pose2d target, double velocity, ChassisSubsystem chassis) {
        this.target  = target;
        this.chassis = chassis;
        this.velocity = velocity;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        Pose2d currentPosition = chassis.getPose();
        Translation2d toTarget = target.getTranslation().minus(currentPosition.getTranslation());
        distanceToTarget = toTarget.getNorm();
        double v = Math.min(velocity, distanceToTarget * kDistance);
        toTarget.times(v/distanceToTarget);
        double omega = toTarget.getAngle().minus(currentPosition.getRotation()).getRadians()*kOmega;
        chassis.setVelocities(new ChassisSpeeds(toTarget.getX(), toTarget.getY(), omega));
    }

    @Override
    public boolean isFinished() {
        return distanceToTarget < MAX_ERROR;
    }


}
