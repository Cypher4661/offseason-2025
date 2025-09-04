// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Examples.DemaciaMotorExample;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.elevator.elevator_subsystems;

public class RobotContainer {

  public static Robot robot;
  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;
  
 // public DemaciaMotorExample demaciaMotorExample = new DemaciaMotorExample();
  public elevator_subsystems elevatorSubsystems = new elevator_subsystems();
  public RobotContainer(Robot robot) {
    RobotContainer.robot = robot;
    RobotContainer.CYCLE_TIME = robot.getPeriod();
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
