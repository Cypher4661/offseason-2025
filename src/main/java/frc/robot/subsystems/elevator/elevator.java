package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private MagneticLimitSwitch limitSwitch;
    private LimitSwitch anotherLimitSwitch;
    private final frc.Demacia.utils.Motors.MotorInterface leftMotor;
    private final frc.Demacia.utils.Motors.MotorInterface rightMotor;
    public  static int floor;
    public elevator(Motor_config leftMotorConfig, Motor_config rightMotorConfig) {
        leftMotor = new frc.Demacia.utils.Motors.TalonMotor(leftMotorConfig.TalonConfig);
        rightMotor = new frc.Demacia.utils.Motors.TalonMotor(rightMotorConfig.TalonConfig);
        this.floor = floor;


    }
    
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      if (limitSwitch.get()) {
        floor = 0;
    
      }
    }


}
