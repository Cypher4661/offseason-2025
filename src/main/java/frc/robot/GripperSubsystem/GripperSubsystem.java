package frc.robot.GripperSubsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.SparkConfig;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.robot.Constants.Gripper; 

public class GripperSubsystem extends SubsystemBase {
  private final SparkMotor motor;
  private final Ultrasonic frontSensor; 
  private final AnalogInput backSensor;  

  public GripperSubsystem() {
    setName("Gripper");

    Ultrasonic.setAutomaticMode(true); 
    frontSensor = new Ultrasonic(Gripper.UltrasonicPingDIO, Gripper.UltrasonicEchoDIO);
    backSensor  = new AnalogInput(Gripper.BackAnalogChannel);

    SparkConfig cfg = new SparkConfig(Gripper.MotorID, "Gripper Motor")
        .withInvert(Gripper.MotorInverted)
        .withBrake(Gripper.MotorBrake)
        .withVolts(Gripper.MaxVoltage)
        .withRampTime(Gripper.RampTime);
    cfg.maxCurrent = Gripper.MaxCurrent;

    motor = new SparkMotor(cfg);
  }

  public void setPower(double power) {
    motor.setDuty(power); }
    
  public void stop() { 
    motor.setDuty(0); }

  public void setNeutralMode(boolean isBrake) {
     motor.setNeutralMode(isBrake); }

  public double getFrontDistanceMeters() {
    return frontSensor.getRangeMM() / 1000.0;
  }

  public double getBackVoltage() {
    return backSensor.getAverageVoltage();
  }

  public boolean isCoralFront() {
    double d = getFrontDistanceMeters();
    if (d <= 0) return false;                      
    if (d > Gripper.MaxDistance) return false;     
    return d < Gripper.FrontMinimumDistance;       
  }

  public boolean isCoralBack() {
    return getBackVoltage() < Gripper.BackMinimumDistance;
  }

  public boolean isCoral() {
    return isCoralFront() && isCoralBack();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("Subsystem");

    builder.addDoubleProperty("FrontDistance(m)", this::getFrontDistanceMeters, null);
    builder.addDoubleProperty("BackVoltage(V)",   this::getBackVoltage,        null);
    builder.addBooleanProperty("IsCoralFront", this::isCoralFront, null);
    builder.addBooleanProperty("IsCoralBack",  this::isCoralBack,  null);
    builder.addBooleanProperty("IsCoral",      this::isCoral,      null);

}}