// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimitSwitch extends SubsystemBase {
  /** Creates a new LimitSwitch. */
  DigitalInput limitSwitch  = new DigitalInput(Constants.elevatorConfig.LimitSwitchID);
  public LimitSwitch() {}
  
  public boolean get(){
    boolean state = limitSwitch.get();
    if (state){
      return false;
    } else {
      return true;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LimitSwitch", get());
    
  }
}
