package frc.robot.subsystems.elevator;

import java.lang.constant.Constable;

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
                    elevator.setPower(Constants.elevatorConfig.CalibreatePowerDown);
                } else {
                    elevator.setPower(Constants.elevatorConfig.CalibreatePowerUp);
                }
                break;
            case Idle:
                elevator.setPower(0);
                break;
            default:
                elevator.setHeight(mode.height);
                break;
        }
    }
}
