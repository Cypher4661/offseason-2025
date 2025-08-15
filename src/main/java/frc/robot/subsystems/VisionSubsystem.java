// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable table;
    private final Field2d field = new Field2d();

    public VisionSubsystem(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    /** מחזיר את המיקום (Pose2d) של הרובוט במגרש */
    public Pose2d getRobotPose() {
        String key = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
                ? "botpose_wpired" : "botpose_wpiblue";

        double[] arr = table.getEntry(key).getDoubleArray(new double[6]);
        if (arr.length < 6) return new Pose2d();
        double x = arr[0];
        double y = arr[1];
        double yawRad = arr[5]; // yaw ברדיאנים
        return new Pose2d(x, y, new Rotation2d(yawRad));
    }

    @Override
    public void periodic() {
        // מעדכן את ה־Field2d בכל מחזור
        field.setRobotPose(getRobotPose());
    }

    public Field2d getField() {
        return field;
    }
}