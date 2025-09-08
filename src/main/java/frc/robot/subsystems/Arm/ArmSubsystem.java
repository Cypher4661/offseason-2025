package frc.robot.subsystems.Arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Motors.TalonConfig;

import frc.Demacia.utils.Sensors.Cancoder;
import frc.Demacia.utils.Sensors.CancoderConfig;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmSubsystem.ArmMode;  


public class ArmSubsystem extends SubsystemBase {

  public enum ArmMode {
    Idle(0),       
    L2(30),      
    L1(-30),     
    Intake(30),
    L3(75),
    L4(105);

    double angle;
    ArmMode(double angle) { this.angle = angle; }
  }

  private final MotorInterface motor;
  private final TalonConfig armConfig;
  private final Cancoder cancoder;            


  private ArmMode mode = ArmMode.Idle; // מצב התחלתי בטוח ,אי הוזזה של הזרוע כברירת מחדל


  public ArmSubsystem() {
    super();
    this.armConfig = Constants.Arm.ARM_CONFIG;
    CancoderConfig cancoderConfig = Constants.Arm.ARM_CANCODER;
    this.motor = new TalonMotor(armConfig);
    this.cancoder = new Cancoder(cancoderConfig);

    setupDashboard();
    setDefaultCommand(new ArmCommand(this));
  }

  public double getAngle() {
    return motor.getCurrentAngle();
  }


  // שליטה ידנית בכוח בטווח של -1 עד 1 
  public void setPower(double percent) {
    motor.setDuty(percent);
  }

  public void stop() {
    setPower(0.0);
  }

 

  public ArmMode getMode() { return mode; }



  public void calibrateFromCancoder() {
   
    double absDegrees = cancoder.getCurrentAbsPosition();
    motor.setEncoderPosition(absDegrees - Constants.Arm.ARM_CANCODER_OFFSET);
  }

public void setAngle(double targetDeg) {
    motor.setMotion(targetDeg);
  }
  

  @Override
  public void periodic() {
    

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Mode", () -> mode.name(), null);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addDoubleProperty("Abs", () -> cancoder.getCurrentAbsPosition(), null);
  }

  private void setupDashboard() {
    MotorCommands.showAngleCommand("Arm: Angle Command", this, motor);
    MotorCommands.showPowerCommand("Arm: Power Command", this, motor);

    SmartDashboard.putData("Arm: Calibrate From Cancoder", new RunCommand(this::calibrateFromCancoder, this).ignoringDisable(true));
    
SmartDashboard.putData("Arm: Idle",
    new InstantCommand(() -> {
      mode = ArmMode.Idle;         
    }, this));

SmartDashboard.putData("Arm: Intake",
    new InstantCommand(() -> {
      mode = ArmMode.Intake;

    }, this));

SmartDashboard.putData("Arm: L1",
    new InstantCommand(() -> {
      mode = ArmMode.L1;

    }, this));

SmartDashboard.putData("Arm: L3",
    new InstantCommand(() -> {
      mode = ArmMode.L3;

    }, this));

SmartDashboard.putData("Arm: Test",
    new InstantCommand(() -> {
      mode = ArmMode.L2;           

    }, this));

SmartDashboard.putData("Arm: L4",
    new InstantCommand(() -> {
      mode = ArmMode.L4;

    }, this));

  }

  

}
