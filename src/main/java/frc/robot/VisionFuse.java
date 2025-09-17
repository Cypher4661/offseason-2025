// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystem;

/** Add your docs here. */
public class VisionFuse{
    public VisionSubsystem reefRightCamera;
    public VisionSubsystem reefLeftCamera;
    public VisionSubsystem feederCamera;

    public VisionFuse(VisionSubsystem reefRightCamera, VisionSubsystem reefLeftCamera, VisionSubsystem feederCamera) {
        this.reefRightCamera = reefRightCamera;
        this.reefLeftCamera = reefLeftCamera;
        this.feederCamera = feederCamera;
    }
}
