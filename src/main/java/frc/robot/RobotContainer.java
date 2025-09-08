package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class RobotContainer {
  // עם Cancoder:
  private final ArmSubsystem arm = new ArmSubsystem(
      Constants.Arm.ARM_CONFIG_DEGREES,
      Constants.Arm.ARM_CANCODER
  );


  public RobotContainer() {
        SmartDashboard.putData("Arm", arm);  

  }

  
}
