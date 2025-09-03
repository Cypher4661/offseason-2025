package frc.robot.subsystems;


import java.io.ObjectInputFilter.Config;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorMotorConfig;
import frc.Demacia.utils.Motors.TalonMotor;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonMotor talonMotor;

  public ElevatorSubsystem (ElevatorMotorConfig config  ) {

    super();
    this.talonMotor = new TalonMotor(config.ElevatorMotorConfig);
    addCommands(config.ratio);
    
  }

  private void addCommands(double ratio) {
    SmartDashboard.putNumber("talon Velocity Target", 1.0);
    SmartDashboard.putData("start talon", new RunCommand(
        () -> {
          setTalonVelocity( SmartDashboard.getNumber("talon Velocity Target", 0.0), ratio);
        }, this));
    SmartDashboard.putData("stop talon", new RunCommand(
        () -> {
          setTalonVelocity(0.0, 1.0);
        }, this));
      }



  public void setTalonVelocity(double velocity, Double ratio) {
    talonMotor.setVelocity(velocity / (ratio));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}







    

