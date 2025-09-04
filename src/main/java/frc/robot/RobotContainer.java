// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ChassisDrive;
import frc.robot.subsystems.Swerve.ChassisSubsystem;

public class RobotContainer {

  public static boolean isRed = true;
  public static Robot robot;
  public CommandXboxController DriverController = new CommandXboxController(0);
  public ChassisSubsystem chassis = new ChassisSubsystem();


  
  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  

  public RobotContainer(Robot robot) {
    RobotContainer.robot = robot;
    RobotContainer.CYCLE_TIME = robot.getPeriod();
    chassis.setDefaultCommand(new ChassisDrive(chassis, DriverController));
    configureBindings();
  }
  
   private void configureBindings() {
  }

  public static boolean isEnabled() {
    return robot.isEnabled();
  }

  public void periodic() {
    N_CYCLE++;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
