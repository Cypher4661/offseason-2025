// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisDrive;
import frc.robot.commands.Autonomous.TuneToReef;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.elevator.ElevatorCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RobotContainer implements Sendable {
  

  public static boolean isRed; 
  public static Robot robot;
  public CommandXboxController DriverController = new CommandXboxController(0);
  public ChassisSubsystem chassis = new ChassisSubsystem();
  public ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(Constants.CAMERA_POSITION, null, chassis);
  private final Field2d field = new Field2d();
  


  
  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  
  

  public RobotContainer(Robot robot) {
    RobotContainer.robot = robot;
    RobotContainer.CYCLE_TIME = robot.getPeriod();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    //chassis.setDefaultCommand(new ChassisDrive(chassis, DriverController)); 
    SmartDashboard.putData("Drive", new ChassisDrive(chassis, elevator, DriverController));
    SmartDashboard.putData("TuneToReef - Left", new TuneToReef(chassis, visionSubsystem, elevator, true));   
    SmartDashboard.putData("TuneToReef - Right", new TuneToReef(chassis, visionSubsystem, elevator, false));   

    SmartDashboard.putData("roboContainer", this);

  
    
    field.setRobotPose(visionSubsystem.getPose());
    configureBindings();
  }
  
   private void configureBindings() {
    DriverController.back().onChange(new InstantCommand(()->chassis.setZeroHeading()).ignoringDisable(true));
    DriverController.a().toggleOnTrue( new InstantCommand(()->chassis.PrecisionMode = !chassis.PrecisionMode));
    DriverController.x().onTrue(new TuneToReef(chassis, visionSubsystem, elevator, true));
    DriverController.b().onTrue(new TuneToReef(chassis, visionSubsystem, elevator, false));

    Trigger driverControllerStickMove = new Trigger(() -> new Translation2d(Math.abs(DriverController.getLeftX()), Math.abs(DriverController.getLeftY())).getNorm() > 0.3);
    driverControllerStickMove.onTrue(new ChassisDrive(chassis, elevator, DriverController));

    chassis.setDefaultCommand(new ChassisDrive(chassis, elevator, DriverController));
    elevator.setDefaultCommand(new ElevatorCommand(elevator));
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      builder.addBooleanProperty("isRed", () -> isRed, p -> isRed = p);
      
  }

  public static boolean isEnabled() {
    return robot.isEnabled();
  }

  public void periodic() {
    if (visionSubsystem.getPose() != null) {
      field.setRobotPose(visionSubsystem.getPose());

  }
  
  }

  public Command getAutonomousCommand() {
    return null;
  }
      public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
  
}
