package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMode;

public class ElevatorCommand extends Command {

    private ElevatorSubsystem elevator;

    public ElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void execute() {
        ElevatorMode mode = elevator.getMode();
        
        switch(mode) {
            case Calibreate:
                if (elevator.getHeight() > mode.height) {
                    mode.height = 0;
                    elevator.setElvPower(Constants.elevatorConfig.CalibreatePowerDown);
                } else {
                    elevator.setElvPower(Constants.elevatorConfig.CalibreatePowerUp);
                }

                break;
            case Idle:
                elevator.setElvPower(0);
                elevator.setArmPower(0);
                break;
            case Intake:
                elevator.setGripperPower(0.25);

                if(elevator.isCoralBack()){
                    elevator.setGripperPower(0);

                }
            default:
                elevator.setHeight(mode.height);
                if(!elevator.elevatorOnly)
                    elevator.setAngle(mode.angle);
                if (elevator.getHeight() > 0.1){
                    
                }
                break;
        }
    }
}
