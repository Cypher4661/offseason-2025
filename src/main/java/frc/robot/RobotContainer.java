// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ChassisDrive;
import frc.robot.commands.Autonomous.TuneToRiff;
import frc.robot.subsystems.Swerve.ChassisSubsystem;

public class RobotContainer {

  public static boolean isRed = SmartDashboard.getBoolean("Is Red", false);
  public static Robot robot;
  public CommandXboxController DriverController = new CommandXboxController(0);
  public ChassisSubsystem chassis = new ChassisSubsystem();


  
  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  

  public RobotContainer(Robot robot) {
    RobotContainer.robot = robot;
    RobotContainer.CYCLE_TIME = robot.getPeriod();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    //chassis.setDefaultCommand(new ChassisDrive(chassis, DriverController)); 
    SmartDashboard.putData("Drive", new ChassisDrive(chassis, DriverController));
    SmartDashboard.putData("Tune in to the Riff", new TuneToRiff(null, CYCLE_TIME, chassis));

    configureBindings();
  }
  
   private void configureBindings() {
    DriverController.back().onChange(new InstantCommand(()->chassis.setZeroHeading()).ignoringDisable(true));
    DriverController.a().toggleOnTrue( new InstantCommand(()->chassis.PrecisionMode = !chassis.PrecisionMode));
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
