package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.Swerve.SwerveConstants.ChassisConstants;

public class ChassisDrive extends Command{
    private ChassisSubsystem chassis;
    private CommandXboxController controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;

    public ChassisDrive(ChassisSubsystem chassis, CommandXboxController controller){
        this.chassis = chassis;
        this.controller = controller;
        addRequirements(chassis);
    }

    @Override
    public void execute(){
        isRed = chassis.isRed();
        direction = isRed ? 1 : -1;
        
        double LjoyX = controller.getLeftY() * direction;
        double LjoyY = controller.getLeftX() * direction;
        //double rot = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
        double rot = controller.getRightX() * -1;
        LjoyY = MathUtil.applyDeadband(LjoyY, ChassisConstants.DeadBand);
        LjoyX = MathUtil.applyDeadband(LjoyX, ChassisConstants.DeadBand);
        rot = MathUtil.applyDeadband(rot, ChassisConstants.DeadBand);

        double VelX = Math.pow(LjoyX, 2) * ChassisConstants.Max_Linear_Speed * Math.signum(LjoyX);
        double VelY = Math.pow(LjoyY, 2) * ChassisConstants.Max_Linear_Speed * Math.signum(LjoyY);
        double VelRot =Math.pow(rot, 2) * ChassisConstants.Max_Rotation_Speed * Math.signum(rot);

        speeds = new ChassisSpeeds(VelX, VelY, VelRot);
            chassis.setVelocities(speeds);
    }    
}
