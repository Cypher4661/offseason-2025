package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMode;

public class ElevatorCommand extends Command {
    ElevatorSubsystem elevator;


    public ElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        ElevatorMode mode = elevator.getMode();
        switch (mode) {
            case Idle:
                elevator.setPower(0);
                return;
            case Calibrate:
                if(elevator.isCalibrated()) {
                    elevator.setMode(ElevatorMode.Idle);
                    elevator.setPower(0);
                    return;
                }
                if(elevator.getHeight() > ElevatorMode.Calibrate.height) {
                    ElevatorMode.Calibrate.height = 0;
                    elevator.setPower(Constants.elevatorConfig.CalibrateDownPower);
                } else {
                    elevator.setPower(Constants.elevatorConfig.CalibrateUpPower);
                }
                return;
            default:
                elevator.setHeight(mode.height);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
