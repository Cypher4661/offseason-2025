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

    private final NetworkTable table;
    private final Field2d field = new Field2d();

    public VisionSubsystem(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);

        Translation2d targetTranslation = new Translation2d(0.0, 0.0);
        field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        field.getObject("robot").setPose(new Pose2d(0.0, 0.0, new Rotation2d()));
        field.setRobotPose(new Pose2d(0.0, 0.0, new Rotation2d()));
        SmartDashboard.putData("Field", field);
    }

    public Pose2d getRobPose() {
        String key = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
                ? "botpose_wpired" : "botpose_wpiblue";

        double[] arr = table.getEntry(key).getDoubleArray(new double[6]);
        if (arr.length < 6) return new Pose2d();
        double x = arr[0];
        double y = arr[1];
        double yawDeg = arr[5];
        return new Pose2d(x, y, new Rotation2d(yawDeg * (Math.PI / 180.0)));
    }

    public Pose2d getRobotPose() {
        Pose2d pose = getRobPose();
        field.setRobotPose(pose);
        return pose;
    }

    /**
     * Returns the current robot pose in the field.
     * 
     * @return The current robot pose.
     */

    public Pose2d getCurrentRobotPose() {
        return field.getRobotPose();
    }

    /**
     * Sets the robot pose in the field.
     * 
     * @param pose The new robot pose to set.
     */

    public void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }


    @Override
    public void periodic() {
        field.setRobotPose(getRobotPose());
    }

    public Field2d getField() {
        return field;
    }
}