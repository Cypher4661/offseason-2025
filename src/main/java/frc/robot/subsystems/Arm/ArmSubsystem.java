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
  private final boolean isDegrees;             
  private final boolean isRadians;

  private ArmMode mode = ArmMode.Idle; // מצב התחלתי בטוח ,אי הוזזה של הזרוע כברירת מחדל
  private boolean holding = false;

  public ArmSubsystem(TalonConfig armConfig, CancoderConfig cancoderConfig) {
    super();
    this.armConfig = armConfig;
    this.motor = new TalonMotor(armConfig);
    this.isDegrees = armConfig.isDegreesMotor;
    this.isRadians = armConfig.isRadiansMotor;
    this.cancoder = (cancoderConfig != null) ? new Cancoder(cancoderConfig) : null;

    setupDashboard();
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

  public void enableHold(boolean enable) {
    this.holding = enable;
    if (enable) {
      moveToAngle(getAngle());
    }
  }


  public void calibrateFromCancoder() {
    if (cancoder == null) return;

    double absRad = cancoder.getCurrentAbsPosition();

    double targetAngle =  Math.toDegrees(absRad) ;

    double currentPosition = motor.getCurrentPosition();

    motor.setEncoderPosition(currentPosition);
  }

private void moveToAngle(double targetDeg) {
    double posNow = motor.getCurrentPosition();   
    double angNow = motor.getCurrentAngle();      
    double delta = targetDeg - angNow;
    delta = ((delta % 360.0) + 360.0) % 360.0;
    motor.setMotion(posNow + delta);
  }
  

  @Override
  public void periodic() {
    if (holding) {
      // החזקה פאסיבית על הזווית הנוכחית (מאפשר תיקונים קטנים נגד גרביטציה/עומס)
      motor.moveToAngle(getAngle());
    }

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Mode", () -> mode.name(), null);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addBooleanProperty("Holding", () -> holding, null);
  }

  private void setupDashboard() {
    MotorCommands.showAngleCommand("Arm: Angle Command", this, motor);
    MotorCommands.showPowerCommand("Arm: Power Command", this, motor);

    SmartDashboard.putData("Arm: Hold", new RunCommand(() -> {
      moveToAngle(getAngle());
      enableHold(true);
    }, this));
    SmartDashboard.putData("Arm: Release Hold", new RunCommand(() -> {
      enableHold(false);
      stop();
    }, this));

    SmartDashboard.putData("Arm: Calibrate From Cancoder", new RunCommand(this::calibrateFromCancoder, this));
    
SmartDashboard.putData("Arm: Idle",
    new InstantCommand(() -> {
      mode = ArmMode.Idle;         
    }, this));

SmartDashboard.putData("Arm: Intake",
    new InstantCommand(() -> {
      mode = ArmMode.Intake;
      moveToAngle(mode.angle);
    }, this));

SmartDashboard.putData("Arm: L1",
    new InstantCommand(() -> {
      mode = ArmMode.L1;
      moveToAngle(mode.angle);
    }, this));

SmartDashboard.putData("Arm: L3",
    new InstantCommand(() -> {
      mode = ArmMode.L3;
      moveToAngle(mode.angle);
    }, this));

SmartDashboard.putData("Arm: Test",
    new InstantCommand(() -> {
      mode = ArmMode.L2;           
      moveToAngle(mode.angle);
    }, this));

SmartDashboard.putData("Arm: L4",
    new InstantCommand(() -> {
      mode = ArmMode.L4;
      moveToAngle(mode.angle);
    }, this));

  }

  

}
