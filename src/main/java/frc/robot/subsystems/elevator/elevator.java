package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;

public class elevator extends SubsystemBase {
    private MagneticLimitSwitch MagneticLimitSwitch; // סוויץ' מגנטי שנדלק בכל קומה
    private DigitalInput limitSwitch  = new DigitalInput(Constants.elevatorConfig.LimitSwitchID);
    DigitalInput magneticLimitSwitch  = new DigitalInput(Constants.elevatorConfig.MagneticLimitSwitchID);
  
    private final MotorInterface leftMotor;
    private final MotorInterface rightMotor;

    // גבהים לכל קומה (במטרים / יחידות אנקודר)
    private static final double[] floorHeights = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}; 
    private static int currentFloor = 0;

    private double previousHeight = 0.0; 
    private int targetFloor = 0;
    private boolean holdingPosition = false;

    // מתח קטן שמחזיק את המעלית במקום (תתאים לפי משקל!)
    private static final double HOLD_VOLTAGE = 0.1; 

    public elevator(Motor_config leftMotorConfig, Motor_config rightMotorConfig) {
        leftMotor = new TalonMotor(leftMotorConfig.TalonConfig);
        rightMotor = new TalonMotor(rightMotorConfig.TalonConfig);
        

        // הגדרת המנוע הימני לעקוב אחרי המנוע השמאלי
        ///בעיהההההההההההההה
        //rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    private boolean isAtButtom(){
        return !limitSwitch.get();
    }
    private boolean IsMagnet(){
        return !limitSwitch.get();
    }


    // פקודה לעבור לקומה
    public void moveToFloor(int targetFloor) {
        if (targetFloor < 0 || targetFloor >= floorHeights.length) {
            System.out.println("Invalid floor: " + targetFloor);
            return;
        }

        this.targetFloor = targetFloor;
        double targetHeight = floorHeights[targetFloor];
        leftMotor.setPositionVoltage(targetHeight); 
        holdingPosition = true;
        System.out.println("Moving to floor: " + targetFloor);
    }

    @Override
    public void periodic() {
        // בדיקה אם ה-Limit Switch בגובה 0 מופעל
        if (isAtButtom()) {
            currentFloor = 0;
            leftMotor.setEncoderPosition(floorHeights[0]);
        }

        // קבלת הגובה הנוכחי מהמנוע
        double currentHeight = leftMotor.getCurrentPosition();

        // זיהוי כיוון התנועה
        boolean isMovingUp = currentHeight > previousHeight;

        // בדיקה אם עברנו קומה לפי מגנטים
        for (int i = 1; i < floorHeights.length; i++) {
            if (Math.abs(currentHeight - floorHeights[i]) < 0.05 && MagneticLimitSwitch.get()) {
                currentFloor = i;
                leftMotor.setEncoderPosition(floorHeights[i]);
                break;
            }
        }

        // אם במצב החזקה – תן מתח קטן
        if (holdingPosition) {
            double error = floorHeights[targetFloor] - currentHeight;

            // אם קרובים ליעד (±2 ס"מ) → החזקה
            if (Math.abs(error) < 0.02) {
                leftMotor.setVoltage(HOLD_VOLTAGE);
            }
        }

        // עדכון הגובה הקודם
        previousHeight = currentHeight;

        // עדכון נתונים ל-SmartDashboard
        SmartDashboard.putNumber("elevator floor", currentFloor);
        SmartDashboard.putNumber("elevator position", leftMotor.getCurrentPosition());
    
    }
}
