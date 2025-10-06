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
public class AutoScore extends SequentialCommandGroup {
  TuneToReef tuneToReef;
  ElevatorSubsystem elevator;
  ChassisSubsystem chassis;

  public AutoScore(boolean goLeft, ElevatorMode mode) {
    this.elevator = RobotContainer.elevator;
    this.chassis = RobotContainer.chassis;
    addCommands(new InstantCommand(()->elevator.setMode(mode)),
    new WaitCommand(3),
      new RunCommand(() -> chassis.setVelocitiesRobotVel(new ChassisSpeeds(0.5, 0, 0)), chassis).until(() -> RobotContainer.visionSubsystem.isSeeTag()),
      new TuneToReef(chassis, RobotContainer.visionSubsystem, elevator, goLeft),
      new RunCommand(()->elevator.setGripperPower(-0.3), elevator).withTimeout(1),
      new RunCommand(()->chassis.setVelocitiesRobotVel(new ChassisSpeeds(-0.5, 0,0)), chassis).withTimeout(2),
      new InstantCommand(()->elevator.setMode(ElevatorMode.Home))    
      );

    

  }
}
