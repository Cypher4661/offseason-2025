// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Camera.CameraType;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotContainer {

    // Subsystems
    private final Camera reefCamera = new Camera(
        "tag",                  
        new Translation3d(-0.123, -0.1175, 0.68),
        0.0,                          // Pitc
        0.0,                            // Yaw
        CameraType.REEF                 
    );
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(reefCamera);


    private final Field2d field = new Field2d();
    // Constructor
    public RobotContainer() {
      // מציג את ה-Field2d בלוח הבקרה
      field.setRobotPose(visionSubsystem.getPose());
      
        configureButtonBindings();
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void periodic() {
      if (visionSubsystem.getPose() != null) {
          field.setRobotPose(visionSubsystem.getPose());
      }
  }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
}
