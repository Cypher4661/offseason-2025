// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ChassisDrive;
import frc.robot.commands.Autonomous.TuneToReef;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RobotContainer {

  public static boolean isRed = SmartDashboard.getBoolean("Is Red", false);
  public static Robot robot;
  public CommandXboxController DriverController = new CommandXboxController(0);
  public ChassisSubsystem chassis = new ChassisSubsystem();
  public ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(Constants.CAMERA_POSITION, null, chassis);
  private final Field2d field = new Field2d();
  private final TuneToReef tuneToReef = new TuneToReef(chassis, visionSubsystem, elevator);


  
  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  
  

  public RobotContainer(Robot robot) {
    RobotContainer.robot = robot;
    RobotContainer.CYCLE_TIME = robot.getPeriod();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    //chassis.setDefaultCommand(new ChassisDrive(chassis, DriverController)); 
    SmartDashboard.putData("Drive", new ChassisDrive(chassis, elevator, DriverController));
    
    field.setRobotPose(visionSubsystem.getPose());
    configureBindings();
  }
  
   private void configureBindings() {
    DriverController.back().onChange(new InstantCommand(()->chassis.setZeroHeading()).ignoringDisable(true));
    DriverController.a().toggleOnTrue( new InstantCommand(()->chassis.PrecisionMode = !chassis.PrecisionMode));
    DriverController.b().toggleOnTrue(new InstantCommand(()->tuneToReef.GoRight = !tuneToReef.GoRight));
    DriverController.x().toggleOnTrue(new InstantCommand(()->tuneToReef.GoLeft = !tuneToReef.GoLeft));


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
