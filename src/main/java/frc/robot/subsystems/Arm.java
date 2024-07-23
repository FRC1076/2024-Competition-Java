package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
public class Arm extends SubsystemBase {
    public final CANSparkMax sprocketLeftMotor;
    public final CANSparkMax sprocketRightMotor;
    
    private final RelativeEncoder sprocketLeftEncoder;
    private final RelativeEncoder sprocketRightEncoder;     //Initialize
    private final DutyCycleEncoder sprocketAbsoluteEncoder;
    ArmFeedforward sprocketFeedforward = new ArmFeedforward(0, ArmConstants.kG, 0);
    PIDController pidController = new PIDController(ArmConstants.kP, 0, 0);
    
    public Arm() {
        sprocketLeftMotor = new CANSparkMax(ArmConstants.sprocketLeftCAN, CANSparkMax.MotorType.kBrushless);
        sprocketRightMotor = new CANSparkMax(ArmConstants.sprocketRightCAN, CANSparkMax.MotorType.kBrushless);
        sprocketLeftMotor.setInverted(ArmConstants.sprocketLeftInversion);
        sprocketRightMotor.setInverted(ArmConstants.sprocketRightInversion);  //Inverts the motor because one motor is on the other side
        sprocketLeftEncoder = sprocketLeftMotor.getEncoder();
        sprocketRightEncoder = sprocketRightMotor.getEncoder();
        
        //declare
        sprocketAbsoluteEncoder = new DutyCycleEncoder(ArmConstants.sprocketAbsEncoderChannel);
    }
    /*Sets the motor speed the desired value and if the speed input is positive, it checks
     * the angle to make sure that the arm does not go above 90 degrees
     */
    public void setSprocketSpeed(double speed) {
        double feedforwardCalculation = sprocketFeedforward.calculate(Math.toRadians(getSprocketAngle()), 0.0);
        if(speed > 0){
            sprocketLimitStop();
        }
        sprocketLeftMotor.set(speed + feedforwardCalculation);
        sprocketRightMotor.set(speed + feedforwardCalculation);
        

    }

    /*
     * Returns the current sprocket angle
     */
    public double getSprocketAngle() {
        return (sprocketAbsoluteEncoder.getAbsolutePosition() * 360 + 20)  % 360 - 49.1;
    }

    /*
     * Returns the current sprocket speed
     */
    public double getSprocketSpeed(){
        return sprocketLeftEncoder.getVelocity();
    }

    /*
     * Moves the sprocket to the desired angle, and if the current sprocket angle
     * is below -30 degrees, it divides the PID Calculation by 2 to ensure a soft approach
     */
    public void sprocketToPosition(double targetPos){
        double sprocketPIDCalculations = pidController.calculate(getSprocketAngle(), targetPos);
        double feedforwardCalculation = sprocketFeedforward.calculate(Math.toRadians(getSprocketAngle()), 0.0);
        if(getSprocketAngle() <= -30){
            sprocketPIDCalculations /= 2;
        }
        double output = sprocketPIDCalculations + feedforwardCalculation;
        sprocketLeftMotor.set(output);
        sprocketRightMotor.set(output);
    
        sprocketLimitStop();
        
    }
/* Stops the sprocket */
    public void stopSprocket(){
        double sprocketFeedforwardCalculation = sprocketFeedforward.calculate(Math.toRadians(getSprocketAngle()), 0);
        sprocketLeftMotor.set(sprocketFeedforwardCalculation);
        sprocketRightMotor.set(sprocketFeedforwardCalculation);
    }
    /*
     * Ensures that the sprocket does not exceed 90 degrees
     */
    public void sprocketLimitStop(){
        if (getSprocketAngle() > 90){
            stopSprocket();
        }

    }
    

}
