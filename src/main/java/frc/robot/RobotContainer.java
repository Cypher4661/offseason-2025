
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisDrive;
import frc.robot.commands.Autonomous.AutoScore;
import frc.robot.commands.Autonomous.AlignAndScore;
import frc.robot.commands.Autonomous.TuneToReef;
import frc.robot.commands.Autonomous.FieldTarget.POSITION;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.Autonomous;
import frc.robot.subsystems.Swerve.ChassisSubsystem;
import frc.robot.subsystems.elevator.ElevatorCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMode;

public class RobotContainer implements Sendable {
  

  public static boolean isRed; 
  public static Robot robot;
  public CommandXboxController DriverController = new CommandXboxController(0);
  public static ChassisSubsystem chassis = new ChassisSubsystem();
  public static ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem(Constants.CAMERA_POSITION, null, chassis);
  private final Field2d field = new Field2d();
  
  


  
  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  
  public static boolean isRed(){
    return isRed;
  }

  public RobotContainer(Robot robot) {
    RobotContainer.robot = robot;
    RobotContainer.CYCLE_TIME = robot.getPeriod();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    //chassis.setDefaultCommand(new ChassisDrive(chassis, DriverController)); 
    SmartDashboard.putData("Drive", new ChassisDrive(chassis, elevator, DriverController));
    SmartDashboard.putData("TuneToReef - Left", new TuneToReef(chassis, visionSubsystem, elevator, true));   
    SmartDashboard.putData("TuneToReef - Right", new TuneToReef(chassis, visionSubsystem, elevator, false));  
    SmartDashboard.putData("roboContainer", this);
    SmartDashboard.putData("Calibrate Elevator", new InstantCommand(()->elevator.setMode(ElevatorMode.Calibreate)));

  
    
    field.setRobotPose(visionSubsystem.getPose());
    configureBindings();
  }
  public static POSITION getClosetPosition(){
    POSITION closet = POSITION.A;
    double minDis = Double.MAX_VALUE;
    for(POSITION pos : POSITION.values()){
      double dis = chassis.getPose().getTranslation().getDistance(pos.getPose().getTranslation());
      if(dis < minDis){
        minDis = dis;
        closet = pos;
      }
    }
    return closet;
  }
  
   private void configureBindings() {
    
    DriverController.rightBumper().onTrue(new InstantCommand(()-> elevator.setMode(ElevatorMode.Intake)));
    DriverController.leftBumper().onTrue(new RunCommand(()->elevator.setGripperPower(0.3)).withTimeout(0.5));
    DriverController.povDown().onTrue(new InstantCommand(()->elevator.setMode(ElevatorMode.Home)));
    DriverController.b().onTrue(new AutoScore(false));
    DriverController.x().onTrue(new AutoScore(true));
    DriverController.back().onChange(new InstantCommand(()->chassis.setZeroHeading()).ignoringDisable(true));
    DriverController.a().toggleOnTrue(new InstantCommand(()->chassis.PrecisionMode = !chassis.PrecisionMode));
    DriverController.y().onTrue(new InstantCommand(()->elevator.setGripperPower(0)));
    DriverController.povUp().onTrue(new InstantCommand(()->{elevator.setMode(ElevatorMode.L1); elevator.setModeElstic(ElevatorMode.L1);}));
    Trigger driverControllerStickMove = new Trigger(() -> new Translation2d(Math.abs(DriverController.getLeftX()), Math.abs(DriverController.getLeftY())).getNorm() > 0.3);
    driverControllerStickMove.onTrue(new ChassisDrive(chassis, elevator, DriverController));
    DriverController.povLeft().onTrue(new RunCommand(()->elevator.setGripperPower(-0.3)).withTimeout(0.5));
    Trigger rumble = new Trigger(()->getTime() <= 15);
    rumble.onTrue(new RunCommand(()->DriverController.setRumble(RumbleType.kBothRumble, 1)).withTimeout(2));
    chassis.setDefaultCommand(new ChassisDrive(chassis, elevator, DriverController));
    elevator.setDefaultCommand(new ElevatorCommand(elevator));
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      builder.addBooleanProperty("isRed", () -> isRed, p -> isRed = p);
      builder.addDoubleProperty("TIMER", this::getTime, null);
      
  }

  public double getTime(){
    return DriverStation.getMatchTime();
  }

  public static boolean isEnabled() {
    return robot.isEnabled();
  }

  public void periodic() {
    if (visionSubsystem.getPose() != null) {
      field.setRobotPose(visionSubsystem.getPose());

  }
  
  }

  public Command getAutonomousCommand() {
    return new Autonomous();
  }
      public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
  
}
