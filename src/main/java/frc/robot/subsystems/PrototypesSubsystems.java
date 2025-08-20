// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ProSparkConfig;
import frc.robot.ProTalonConfig;
import frc.Demacia.utils.Motors.SparkConfig;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.Demacia.utils.Motors.TalonMotor;


public class PrototypesSubsystems extends SubsystemBase {
  /** Creates a new PrototypesSubsystems. */
private final frc.Demacia.utils.Motors.TalonMotor talonMotor;
private final frc.Demacia.utils.Motors.SparkMotor sparkMotor;

  public PrototypesSubsystems(ProTalonConfig config ,ProSparkConfig SparkConfig ) {

    super();
    this.talonMotor = new TalonMotor(config.ProTalonConfig);
    addCommands(config.ratio);
    
    this.sparkMotor = new SparkMotor(SparkConfig.ProSparkConfig);
    addSparkCommands(SparkConfig.SparkGearRatio);
  }

  private void addCommands(double ratio) {
    SmartDashboard.putNumber("talon Velocity Target", 1.0);
    SmartDashboard.putData("start talon", new RunCommand(
        () -> {
          setTalonVelocity( SmartDashboard.getNumber("talon Velocity Target", 0.0), ratio);
        }, this));
    SmartDashboard.putData("stop talon", new RunCommand(
        () -> {
          setTalonVelocity(0.0, 1.0);
        }, this));
      }


  private void addSparkCommands(double SparkGearRatio) {
    SmartDashboard.putNumber("spark Velocity Target", 1.0);
    SmartDashboard.putData("start spark", new RunCommand(
      () -> {
        try {
          setSparkVelocity(SmartDashboard.getNumber("spark Velocity Target", 0.0), SparkGearRatio);
        } catch (Exception e) {
          // TODO: handle exception
          SmartDashboard.putString("Error", "Failed to set spark velocity: " + e.getMessage());
          System.out.println("Failed to set spark velocity: " + e.getMessage());
        }
        
      }, this));
   SmartDashboard.putData("stop spark", new RunCommand(
      () -> {
        setSparkVelocity(0.0, 1.0);
      }, this));

  }

  public void setTalonVelocity(double velocity, Double ratio) {
    talonMotor.setVelocity(velocity / (ratio));
    SmartDashboard.putNumber("Talon Target Velocity", velocity);
    SmartDashboard.putNumber("Talon Actual Velocity", talonMotor.getCurrentVelocity());
  }
  public void setSparkVelocity(double velocity, Double ratio) {
    sparkMotor.setVelocity(velocity / (ratio));
    SmartDashboard.putNumber("Spark Target Velocity", velocity);
    SmartDashboard.putNumber("Spark Actual Velocity", sparkMotor.getCurrentVelocity());
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Talon Current Velocity", talonMotor.getCurrentVelocity());
    SmartDashboard.putNumber("Spark Current Velocity", sparkMotor.getCurrentVelocity());
    // This method will be called once per scheduler run
  }
}
