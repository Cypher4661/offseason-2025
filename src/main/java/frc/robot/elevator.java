package frc.robot;

public class elevator {
    private final frc.Demacia.utils.Motors.MotorInterface leftMotor;
    private final frc.Demacia.utils.Motors.MotorInterface rightMotor;
    
    public elevator(Motor_config leftMotorConfig, Motor_config rightMotorConfig) {
        leftMotor = new frc.Demacia.utils.Motors.TalonMotor(leftMotorConfig.TalonConfig);
        rightMotor = new frc.Demacia.utils.Motors.TalonMotor(rightMotorConfig.TalonConfig);


    }


}
