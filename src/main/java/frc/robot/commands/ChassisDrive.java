package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveConstants.ChassisConstants;

public class ChassisDrive extends Command{
    private ChassisSubsystem chassis;
    private CommandXboxController controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;
    private static boolean precisionMode;

    public ChassisDrive(ChassisSubsystem chassis, CommandXboxController controller){
        this.chassis = chassis;
        this.controller = controller;
        ChassisDrive.precisionMode = false;
        addRequirements(chassis);
    }

    public static boolean getPecisionMode(){
        return precisionMode;
    }

    //public static void setPrecisitinMode(boolean precisionMode){
      //  ChassisDrive.precisionMode = precisionMode;
        //if(!precisionMode){
          //  RobotContainer.robot.setNotPrecisionMode();
        //}
    //}

    //public void invertPrecisionMode(){
      //  setPrecisitinMode(!precisionMode);
    //}

    @Override
    public void execute(){
        isRed = chassis.isRed();
        direction = isRed ? 1 : -1;
        double LjoyY = controller.getLeftY() * direction;
        double LjoyX = controller.getLeftX() * direction;
        double rot = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();

        double VelX = Math.pow(LjoyX, 2) * ChassisConstants.Max_Linear_Speed * Math.signum(LjoyX);
        double VelY = Math.pow(LjoyY, 2) * ChassisConstants.Max_Linear_Speed * Math.signum(LjoyY);
        double VelRot =Math.pow(rot, 2) * ChassisConstants.Max_Rotation_Speed * Math.signum(rot);

        speeds = new ChassisSpeeds(VelX, VelY, VelRot);

        if(precisionMode){
           chassis.setVelocities(speeds);
        }
        else{
            chassis.setVelocityWithAccel(speeds);
        }
    }    
    


}
