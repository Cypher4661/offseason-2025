// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public VisionSubsystem() {}

    public Pose2d getEstimatedPose() {
        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

        // הגנה במקרה שאין נתונים תקינים
        if (botpose.length < 6) {
            return new Pose2d(); // מחזיר מיקום אפס
        }

        return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("Robot Pose");

        builder.addDoubleProperty("Limelight X", () -> getEstimatedPose().getX(), null);
        builder.addDoubleProperty("Limelight Y", () -> getEstimatedPose().getY(), null);
        builder.addDoubleProperty("Limelight Angle", () -> getEstimatedPose().getRotation().getDegrees(), null);
    }

    @Override
    public void periodic() {
        // אם תרצה לכתוב עוד דברים בעתיד, זה המקום
    }
}
