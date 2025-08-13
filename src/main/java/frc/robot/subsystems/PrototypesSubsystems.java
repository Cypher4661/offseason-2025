// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ProTalonConfig;
import frc.robot.utils.TalonMotor;

public class PrototypesSubsystems extends SubsystemBase {
  /** Creates a new PrototypesSubsystems. */
  private final TalonMotor talonMotor;
  public PrototypesSubsystems(ProTalonConfig config) {

    super();
    this.talonMotor = new TalonMotor(config.ProTalonConfig);
    addCommands();
  }

  private void addCommands() {
    SmartDashboard.putNumber("talon Velocity Target", 1.0);
    SmartDashboard.putData("start talon", new RunCommand(
        () -> {
          setTalonVelocity( SmartDashboard.getNumber("talon Velocity Target", 0.0));
        }, this));
    SmartDashboard.putData("stop talon", new RunCommand(
        () -> {
          setTalonVelocity(0.0);
        }, this));


  }
  public void setTalonVelocity(double velocity) {
    talonMotor.setVelocity(velocity / (1.0/100));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
