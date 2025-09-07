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
    Idle(0),      // לא עושה כלום
    L2(0),      
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

  public void setAngle(double angle) {
    motor.setAngle(angle);
  }

  // שליטה ידנית בכוח בטווח של -1 עד 1 
  public void setPower(double percent) {
    motor.setDuty(percent);
  }

  public void stop() {
    setPower(0.0);
  }

  public void setMode(ArmMode newMode) {
    this.mode = safeParseMode(newMode.name());
    if (this.mode != ArmMode.Idle) {
      setAngle(this.mode.angle);
      holding = true;
    }
  }

  public void setMode(String modeName) {
    setMode(safeParseMode(modeName));
  }

  public ArmMode getMode() { return mode; }

  public void enableHold(boolean enable) {
    this.holding = enable;
    if (enable) {
      setAngle(getAngle());
    }
  }


  public void calibrateFromCancoder() {
    if (cancoder == null) return;

    double absRad = cancoder.getCurrentAbsPosition();

    double targetAngle =  Math.toDegrees(absRad) ;

    double currentPosition = motor.getCurrentPosition();
    double currentAngle = motor.getCurrentAngle();
    double deltaAngle = normalizeAngleForUnits(targetAngle - currentAngle);

    double newPosition = currentPosition + deltaAngle; 
    motor.setEncoderPosition(newPosition);
  }

  @Override
  public void periodic() {
    if (holding) {
      // החזקה פאסיבית על הזווית הנוכחית (מאפשר תיקונים קטנים נגד גרביטציה/עומס)
      motor.setAngle(getAngle());
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

    // Hold/Release
    SmartDashboard.putData("Arm: Hold", new RunCommand(() -> {
      setAngle(getAngle());
      enableHold(true);
    }, this));
    SmartDashboard.putData("Arm: Release Hold", new RunCommand(() -> {
      enableHold(false);
      stop();
    }, this));

    SmartDashboard.putData("Arm: Calibrate From Cancoder", new RunCommand(this::calibrateFromCancoder, this));
    

SmartDashboard.putData("Arm: Idle",
    new InstantCommand(() -> setMode(ArmMode.Idle), this));
SmartDashboard.putData("Arm: Intake",
    new InstantCommand(() -> setMode(ArmMode.Intake), this));
SmartDashboard.putData("Arm: L1",
    new InstantCommand(() -> setMode(ArmMode.L1), this));
SmartDashboard.putData("Arm: L3",
    new InstantCommand(() -> setMode(ArmMode.L3), this));
SmartDashboard.putData("Arm: Test",
    new InstantCommand(() -> setMode(ArmMode.L2), this));
SmartDashboard.putData("Arm: L4",
    new InstantCommand(() -> setMode(ArmMode.L4), this));
  }

  private ArmMode safeParseMode(String s) {
    try { return ArmMode.valueOf(s); }
    catch (Exception e) { return ArmMode.L2; }
  }

  private double normalizeAngleForUnits(double delta) {
    if (isRadians) {
      while (delta > Math.PI) delta -= 2*Math.PI;
      while (delta <= -Math.PI) delta += 2*Math.PI;
      return delta;
    }
    if (isDegrees) {
      // לטווח (-180, 180]
      while (delta > 180.0) delta -= 360.0;
      while (delta <= -180.0) delta += 360.0;
      return delta;
    }
    return delta;
  }
}
