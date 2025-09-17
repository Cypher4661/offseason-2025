package frc.robot.subsystems.Arm;

import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.SparkConfig;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.Demacia.utils.Motors.TalonConfig;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {




private static final String SensorConstants = null;
private final SparkMotor motor;
  private final Ultrasonic upSensor;
  private final AnalogInput downSensor;
private SparkConfig armConfig;

public  GripperSubsystem () {
super();
    this.armConfig = Constants.Arm.עדיין ריק;
    this.motor = new SparkMotor(עדיין ריק);


    upSensor = new Ultrasonic(SensorConstants.UP_SENSOR_CHANNELS.getFirst(), SensorConstants.UP_SENSOR_CHANNELS.getSecond());
    downSensor = new AnalogInput(SensorConstants.DOWN_SENSOR_CHANNEL);
  }

    
}
