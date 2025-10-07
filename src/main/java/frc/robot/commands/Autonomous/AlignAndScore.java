// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.lang.annotation.ElementType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndScore extends SequentialCommandGroup {
  TuneToReef tuneToReef;
  ElevatorSubsystem elevator;
  ChassisSubsystem chassis;

  public AlignAndScore(boolean goLeft, ElevatorMode mode) {
    this.elevator = RobotContainer.elevator;

    this.chassis = RobotContainer.chassis;
    addCommands(
      new InstantCommand(()->elevator.setMode(mode)),
      new RunCommand(()->chassis.setVelocities(new ChassisSpeeds())).withTimeout(1.5),
      new TuneToReef(chassis, RobotContainer.visionSubsystem, elevator, goLeft),
      new RunCommand(()->chassis.setVelocities(new ChassisSpeeds())).withTimeout(0.5),
      
       new RunCommand(()->elevator.setGripperPower(elevator.getMode() == ElevatorMode.L2? 0.3 : -0.3)).withTimeout(1.5),
       new InstantCommand(()->elevator.setGripperPower(0)),
      new RunCommand(()->chassis.setVelocitiesRobotRel(new ChassisSpeeds(-1, 0,0))).withTimeout(1),
      new InstantCommand(()->elevator.setMode(ElevatorMode.Home))    
      );

    

  }
}
