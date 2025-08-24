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
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry tv;

    public Pose2d pose;

    public VisionSubsystem(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);  
        double v = tv.getDouble(0.0);
        if(v != 0) {
            
        }
    }
}