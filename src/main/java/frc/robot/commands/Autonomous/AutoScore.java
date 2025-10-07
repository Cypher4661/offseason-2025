// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.Autonomous.FieldTarget.POSITION;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {
  Pose2d poseWithOffset = Pose2d.kZero;
  POSITION position;
  boolean goLeft;
  ChassisSubsystem chassis;
  public AutoScore(boolean goLeft) {
    this.goLeft = goLeft;
    this.chassis = RobotContainer.chassis;
  }
  @Override
  public void initialize() {
    this.position = RobotContainer.getClosetPosition();
    ElevatorMode mode = RobotContainer.elevator.getModeElstic();
    poseWithOffset = new Pose2d(position.getPose().getTranslation().plus(new Translation2d(1, position.getPose().getRotation())), position.getPose().getRotation().plus(Rotation2d.k180deg));
    new SequentialCommandGroup(
      
    new GoToCommand(poseWithOffset, 2, chassis),
    new RunCommand(()-> chassis.setVelocitiesRobotVel(new ChassisSpeeds(1, 0, 0))).until(()->RobotContainer.visionSubsystem.isSeeTag() || chassis.getPose().getTranslation().getDistance(position.getPose().getTranslation()) < 0.5).withTimeout(3),

    new AlignAndScore(goLeft, mode)).schedule();
    
  }
  @Override
  public boolean isFinished() {
      return true;
  }

}
