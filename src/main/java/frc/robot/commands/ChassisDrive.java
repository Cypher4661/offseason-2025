package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Builder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveConstants.ChassisConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ChassisDrive extends Command{
    private ChassisSubsystem chassis;
    private CommandXboxController controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;
    private boolean UseStick;
    ElevatorSubsystem Elevator;
    private double Kheight = 1;
    
    public ChassisDrive(ChassisSubsystem chassis, ElevatorSubsystem elevator, CommandXboxController controller){
        this.chassis = chassis;
        this.controller = controller;
        this.Elevator = elevator;
        addRequirements(chassis);
        SmartDashboard.putBoolean("Use Stick", false);
        SmartDashboard.putNumber("Kheight", Kheight);
        
    }


    @Override
    public void execute(){
        isRed = chassis.isRed();
        direction = isRed ? 1 : -1;
        UseStick = SmartDashboard.getBoolean("Use Stick", false);

        double rot = 0;
        double LjoyX = controller.getLeftY() * direction;
        double LjoyY = controller.getLeftX() * direction;
        if(UseStick){
            rot = controller.getRightX() * direction;
        } else {
            rot  = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
        }

        Kheight = -(0.45 * Elevator.getHeight() - 0.1);
        System.out.println(Kheight);
        

        LjoyY = MathUtil.applyDeadband(LjoyY, ChassisConstants.DeadBand);
        LjoyX = MathUtil.applyDeadband(LjoyX, ChassisConstants.DeadBand);
        rot = MathUtil.applyDeadband(rot, ChassisConstants.DeadBand);

        double multiplier = chassis.PrecisionMode ? SwerveConstants.ChassisConstants.PrecisionModeMultiplier : 1;
        double VelX = Math.pow(LjoyX, 2) * multiplier * Math.signum(LjoyX) * ChassisConstants.Max_Linear_Speed;// * Kheight;
        double VelY = Math.pow(LjoyY, 2) * multiplier * Math.signum(LjoyY) * ChassisConstants.Max_Linear_Speed;// * Kheight;
        double VelRot =Math.pow(rot, 2) * multiplier * Math.signum(rot) * ChassisConstants.Max_Rotation_Speed;// * Kheight;
        speeds = new ChassisSpeeds(VelX, VelY, VelRot);
        chassis.setVelocities(speeds);

        }
        
        /* 

        if (chassis.PrecisionMode){
            double VelX = Math.pow(LjoyX, 2) * 0.7 * Math.signum(LjoyX);
            double VelY = Math.pow(LjoyY, 2) * 0.7 * Math.signum(LjoyY);
            double VelRot =Math.pow(rot, 2) * 0.7 * Math.signum(rot);
            speeds = new ChassisSpeeds(VelX, VelY, VelRot);
            chassis.setVelocities(speeds);
            //System.out.println("Precision Mode");
            
        }
        else{
            double VelX = Math.pow(LjoyX, 2) * ChassisConstants.Max_Linear_Speed * Math.signum(LjoyX);
            double VelY = Math.pow(LjoyY, 2) * ChassisConstants.Max_Linear_Speed * Math.signum(LjoyY);
            double VelRot =Math.pow(rot, 2) * ChassisConstants.Max_Rotation_Speed * Math.signum(rot);
            speeds = new ChassisSpeeds(VelX, VelY, VelRot);
            chassis.setVelocities(speeds);
            //System.out.println("Normal Mode");
            
        }
        */
        
        

        
    }  

