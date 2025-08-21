// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable table;
    private final Field2d field;
    private double tagX;
    private double tagY;
    private double id;
    public Pose2d pose;

    public VisionSubsystem(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
        field = new Field2d();
        
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        tagX = table.getEntry("tx").getDouble(0.0);
        tagY = -(table.getEntry("ty").getDouble(0.0));
        
    }
    public double DistToTarget() {
        dist = (Math.abs(height - camera.getPitch())) * (Math.tan(Math.toRadians(tagX)));
    }