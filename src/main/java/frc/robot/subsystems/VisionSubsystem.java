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

    /** Gets robot pose from Limelight in field coordinates. */
    public Pose2d getEstimatedPose() {
        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        return new Pose2d();
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
    }
}
