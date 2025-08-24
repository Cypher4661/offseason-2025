// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // Subsystems
  private final VisionSubsystem visionSubsystem = new VisionSubsystem("limelight");

    // מציג את ה-Field2d של ה-Vision בלוח הבקרה
  public RobotContainer(Robot robot) {

    configureButtonBindings();
  }
  private void configureButtonBindings() {

    // Configure your button bindings here

  }
  public Command getAutonomousCommand() {
    // Return the command to run in autonomous mode
    return null; // Replace with your autonomous command
  }
  public void periodic() {
    // Update the Field2d with the current pose of the vision subsystem
  }
}
