package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void Move(double highet) {
        rightMotor.setPositionVoltage(highet);
        
    }
    
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      if (limitSwitch.get()) {
        floor = 0;
    
      }
      SmartDashboard.putNumber("elevator floor", floor);
      SmartDashboard.putNumber("elevator position", rightMotor.getCurrentPosition());
    }


}
