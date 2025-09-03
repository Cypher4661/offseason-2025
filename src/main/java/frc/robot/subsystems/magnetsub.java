// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class magnetsub extends SubsystemBase {
  /** Creates a new magnetsub. */
  DigitalInput test = new DigitalInput(0);
  private double count;
  private boolean counting;

  public magnetsub() {
    SmartDashboard.putData("Magnet Sensor", test);
    this.count = 0;
    this.counting = false;
   
    

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (test.get()){
      SmartDashboard.putBoolean("Is there a magnet", false);
      this.counting = false;

    } else {
      SmartDashboard.putBoolean("Is there a magnet", true);
      if (!this.counting){
        this.counting = true;
        this.count++;
      }
      
    }
    if (this.count > SmartDashboard.getNumber("Magnet Count Reset", 10)){

      this.count = 0;
      
    }
    SmartDashboard.getNumber("Magnet Count Reset", 10);
    SmartDashboard.putNumber("Magnet Count", this.count);

  }
}
