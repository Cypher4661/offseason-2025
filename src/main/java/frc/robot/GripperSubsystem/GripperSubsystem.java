package frc.robot.GripperSubsystem;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.SparkConfig;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.robot.Constants.Gripper;
import frc.Demacia.utils.Log.LogEntry;
import frc.Demacia.utils.Log.LogManager;
import frc.Demacia.utils.Log.LogSupplier;
import frc.Demacia.utils.Log.LogManager.LOG_TARGET;

import static frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
  private final SparkMotor motor;
  private final Ultrasonic frontsensor;
  private final AnalogInput backsensor;

  public GripperSubsystem() {
    setName("Gripper");

    Ultrasonic.setAutomaticMode(true); 
    frontsensor   = new Ultrasonic(Gripper.UltrasonicPingDIO, Gripper.UltrasonicEchoDIO); // לא הבנתי מה זה
    backsensor = new AnalogInput(Gripper.BackAnalogChannel);// לא הבנתי מה זה אומר

    SparkConfig cfg = new SparkConfig(Gripper.MotorID, "Gripper Motor")
        .withInvert(Gripper.MotorInverted)
        .withBrake(Gripper.MotorBrake)
        .withVolts(Gripper.MaxVoltage)
        .withRampTime(Gripper.RampTime);
    cfg.maxCurrent = Gripper.MaxCurrent;

    motor = new SparkMotor(cfg);  

    addNT();
  }

  public void setPower(double power) { motor.setDuty(power); }
  public void stop() { motor.setDuty(0); }
  public void setNeutralMode(boolean isBrake) { motor.setNeutralMode(isBrake); }

  public double getfrontDistanceMeters() {
    return frontsensor.getRangeMM() / 1000.0;
  }

  public double getbackVoltage() {
    return backsensor.getAverageVoltage();
  }

  public boolean isCoralFront() {
    double d = getfrontDistanceMeters();
    if (d <= 0) return false;                               
    if (d > Gripper.MaxDistance) return false;    
    return d < Gripper.FrontMinimumDistance;              
  }

  public boolean isCoralBack() {
    return getbackVoltage() < Gripper.BackMinimumDistance;
  }

  public boolean isCoral() {
    return isCoralFront() && isCoralBack();
  }

  private void addNT() {
    LogManager.addEntry(getName() + "/get up sensor", this::getfrontSensor, 3);
    LogManager.addEntry(getName() + "/get down sensor", this::getbackSensor, 3);
    LogManager.addEntry(getName() + "/Is Coral", this::isCoral, 4);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public void periodic() {
  }
}
