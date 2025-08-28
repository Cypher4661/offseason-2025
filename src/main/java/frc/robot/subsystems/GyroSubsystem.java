// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  private AHRS gyro;
  
  /** Creates a new gyro. */
  public GyroSubsystem() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Yaw", getYaw());
    // This method will be called once per scheduler run
  }

  public double getYaw() {
    return gyro.getYaw();
  }
}
