package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double UseStike;
    private double UseTrigger;
    private double PrecisionMode;

    public ChassisDrive(ChassisSubsystem chassis, CommandXboxController controller){
        this.chassis = chassis;
        this.controller = controller;
        addRequirements(chassis);
        SmartDashboard.putBoolean("Use Stike", false);
        SmartDashboard.putBoolean("Is Precision Mode", false);
        
    }

    @Override
    public void execute(){
        isRed = chassis.isRed();
        direction = isRed ? 1 : -1;
        UseStike = SmartDashboard.getBoolean("Use Stike", false)? 1 : -1;
        PrecisionMode = SmartDashboard.getBoolean("Is Precision Mode", false)? 1 : -1;

        double rot = 0;
        double LjoyX = controller.getLeftY() * direction;
        double LjoyY = controller.getLeftX() * direction;
        if(UseStike == 1){
            rot = controller.getRightX() * direction;
        }
        else {
            rot  = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
        }

        LjoyY = MathUtil.applyDeadband(LjoyY, ChassisConstants.DeadBand);
        LjoyX = MathUtil.applyDeadband(LjoyX, ChassisConstants.DeadBand);
        rot = MathUtil.applyDeadband(rot, ChassisConstants.DeadBand);



        // if (PrecisionMode == 1){
            
        // }
        // else{
            
        // }

        
    }    
}
