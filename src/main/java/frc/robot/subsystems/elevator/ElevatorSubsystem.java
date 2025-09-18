package frc.robot.subsystems.elevator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Log.LogManager;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Sensors.Cancoder;
import frc.robot.Constants;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSubsystem extends SubsystemBase {
    // סוויץ' מגנטי שנדלק בכל קומה

    private DigitalInput buttomSwitch  = new DigitalInput(Constants.elevatorConfig.LimitSwitchID);
    private DigitalInput magenticSwitch  = new DigitalInput(Constants.elevatorConfig.MagneticLimitSwitchID);
  
    private final MotorInterface leftMotor;
    private final MotorInterface rightMotor;
    private final MotorInterface armMotor;
    private final Cancoder cancoder;            
    private double armOffset = Constants.Arm.ARM_CANCODER_OFFSET;
    boolean elevatorOnly = false;


    // גבהים לכל קומה (במטרים / יחידות אנקודר)
    private static final double[] magenticHeights = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}; 

    public enum ElevatorMode { 
        Idle(0, -90), 
        Home(0, -90), 
        Intake(0.2, 0), 
        L1(0.3, -20), 
        L2(0.7, 20), 
        L3(0.9, 50), 
        L4(1.2,70), 
        AlgieUp(1,60), 
        AlgieDown(0.4, 30), 
        Barge(1.2, 90), 
        Test(0,0),
        Calibreate(0, -90);
        double height;
        double angle;
        private ElevatorMode(double height, double angle) {this.height = height; this.angle = angle; }
    }

    private ElevatorMode mode = ElevatorMode.Calibreate;
    private boolean calibreated = false;
    private double minHeight = 0;
    private double maxHeight = 1.2;

    public ElevatorSubsystem() {
        LogManager.log("ElevatorSubsystem started");
        leftMotor = new TalonMotor(Constants.elevatorConfig.LeftMotor);
        rightMotor = new TalonMotor(Constants.elevatorConfig.RightMotor);
        ((TalonFX)rightMotor).setControl(new Follower(Constants.elevatorConfig.LeftMotor.id, true));
        armMotor = new TalonMotor(Constants.Arm.ARM_CONFIG);
        cancoder = new Cancoder(Constants.Arm.ARM_CANCODER);
        calibrateFromCancoder();

        // start mode
        mode = ElevatorMode.Calibreate;
        mode.height = getHeight() + 0.05;
        
        // Elastic 
        leftMotor.showConfigPIDFSlotCommand(0);
        leftMotor.showConfigMotionVelocitiesCommand();
        armMotor.showConfigPIDFSlotCommand(0);
        armMotor.showConfigMotionVelocitiesCommand();

        MotorCommands.showPowerCommand("Elevator Power", this, leftMotor);
        MotorCommands.showPositionCommand("Elevator Position", this, leftMotor);
        MotorCommands.showPowerCommand("Arm Power", this, armMotor);
        MotorCommands.showAngleCommand("Arm Angle cmd", this, armMotor);
        SmartDashboard.putData("Elecator Command",new ElevatorCommand(this));
        // setDefaultCommand(new ElevatorCommand(this));
        SmartDashboard.putData("Elevetor", this);
    }
    

    private boolean isAtButtom(){
        return !buttomSwitch.get() || getHeight() <= minHeight;
        
    }
    private boolean IsMagnet(){
        return !magenticSwitch.get();
    }

    public double getHeight(){
        return leftMotor.getCurrentPosition();
    } 

    public double getAngle(){
        return armMotor.getCurrentAngle();
    }
    public double getAbsAngle(){
        return cancoder.getCurrentAbsPosition();
    }
    

    public void setArmPower(double percent) {
        armMotor.setDuty(percent);
    }

    public void stopArm() {
        setArmPower(0.0);
    }

    public void calibrateFromCancoder() {
        double absDegrees = cancoder.getCurrentAbsPosition();
        armMotor.setEncoderPosition(absDegrees - armOffset);
    }

    public void setAngle(double targetDeg) {
        armMotor.setMotion(targetDeg);
    }

    public void setElvPower(double power) {
        if(power < 0 && isAtButtom()) power = 0;
        if(power > 0 && getHeight() > maxHeight) power = 0;
        leftMotor.setDuty(power);
    }

    public void setHeight(double height) {
        if(!calibreated) return;
        height = Math.max(minHeight, Math.min(maxHeight, height));
        leftMotor.setPositionVoltage(height, isAtButtom()? 0 :  Constants.elevatorConfig.kg);
    }

    public void setMode(ElevatorMode mode) {
        if(!calibreated) {
            this.mode = ElevatorMode.Calibreate;
        } else {
            this.mode = mode;
        }   
    }
    public void setMode(String mode) {
        try{
            setMode(ElevatorMode.valueOf(mode));
        } catch(Exception e){
            System.out.println("Invalid elevator mode: " + mode);
        }
    }

    public ElevatorMode getMode() {
        return mode;
    }

    public String getModeString() {
        return mode.toString();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("angle", getAngle());
        if(isAtButtom()){
            calibreated = true;
            leftMotor.setEncoderPosition(minHeight);
            setElvPower(0);
        } else if(calibreated && IsMagnet()) {
            double h = getHeight();
            double closest = 0;
            double error = Double.MAX_VALUE;
            for(double mh : magenticHeights){
                if(Math.abs(mh - h) < error){
                    error = Math.abs(mh - h);
                    closest = mh;
                }
            }
            LogManager.log("Elevator magnet at height " + closest + " corrected " + (closest - h));
            leftMotor.setEncoderPosition(closest);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    
        builder.addStringProperty("Mode", this::getModeString, this::setMode);
        builder.addDoubleProperty("Height", this::getHeight, null);
        builder.addBooleanProperty("Calibreated", () -> calibreated, null);
        builder.addBooleanProperty("buttom", this::isAtButtom, null);
        builder.addBooleanProperty("magnet", this::IsMagnet, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("AbsAngle", this::getAbsAngle, null);
        
        builder.addDoubleProperty("Test Height", ()->ElevatorMode.Test.height, (height)->ElevatorMode.Test.height = height);
        builder.addDoubleProperty("Test Angle", ()->ElevatorMode.Test.angle, (angle)->ElevatorMode.Test.angle = angle);
        builder.addBooleanProperty("Elevator Only", ()->elevatorOnly, (b)->elevatorOnly = b);
        builder.addDoubleProperty("Arm Offset", ()->armOffset, (o)->{armOffset = 0; calibrateFromCancoder();});
    }
}
