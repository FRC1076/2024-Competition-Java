package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Arm extends SubsystemBase {
    public final CANSparkMax sprocketLeftMotor;
    public final CANSparkMax sprocketRightMotor;
    private final SparkPIDController sprocketLeftPIDController;
    private final SparkPIDController sprocketRightPIDController;
    private final RelativeEncoder sprocketLeftEncoder;
    private final RelativeEncoder sprocketRightEncoder;     //Initialize
    private final DutyCycleEncoder sprocketAbsoluteEncoder;
    ArmFeedforward sprocketFeedforward = new ArmFeedforward(0, 0.0275, 0);
    PIDController pidController = new PIDController(Constants.armConstants.controllerGain, 0, 0);
    
    public Arm() {
        sprocketLeftMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
        sprocketRightMotor = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
        sprocketRightMotor.setInverted(true);  //Inverts the motor because one motor is on the other side
        sprocketLeftEncoder = sprocketLeftMotor.getEncoder();
        sprocketRightEncoder = sprocketRightMotor.getEncoder();
        sprocketLeftPIDController = sprocketLeftMotor.getPIDController();
        sprocketRightPIDController = sprocketRightMotor.getPIDController();
        sprocketLeftPIDController.setP(Constants.armConstants.controllerGain);
        sprocketRightPIDController.setP(Constants.armConstants.controllerGain); //declare
        sprocketAbsoluteEncoder = new DutyCycleEncoder(0);
    }
    /*Sets the motor speed the desired value and if the speed input is positive, it checks
     * the angle to make sure that the arm does not go above 90 degrees
     */
    public void setSprocketSpeed(double speed) {
        double feedforwardcalculation = sprocketFeedforward.calculate(Math.toRadians(getSprocketAngle()),0.0);
        sprocketLeftMotor.set(speed+feedforwardcalculation);
        sprocketRightMotor.set(speed+feedforwardcalculation);
        if(speed>0){
        sprocketLimitStop();
        }

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
     * Makes the sprocket full speed down
     */
    public void sprocketfullspeeddown(){
        sprocketLeftMotor.set(-1);
        sprocketRightMotor.set(-1);
    }
    /*
     * Makes the sprocket full speed up
     */
    public void sprocketfullspeedup(){
        sprocketLeftMotor.set(1);
        sprocketRightMotor.set(1);
    }
    /*
     * Moves the sprocket to the desired angle, and if the current sprocket angle
     * is below -30 degrees, it divides the PID Calculation by 2 to ensure a soft approach
     */
    public void sprocketToPosition(double targetPos){
        double SprocketPIDCalculations = pidController.calculate(getSprocketAngle(), targetPos);
        double feedforwardcalculation = sprocketFeedforward.calculate(Math.toRadians(getSprocketAngle()),0.0);
        if(getSprocketAngle() <= -30){
            SprocketPIDCalculations /= 2;
        }
        double output = SprocketPIDCalculations + feedforwardcalculation;
        sprocketLeftMotor.set(output);
        sprocketRightMotor.set(output);
    
        sprocketLimitStop();
        
    }
/* Stops the sprocket */
    public void stopSprocket(){
        double sprocketFeedforwardcalculation = sprocketFeedforward.calculate(Math.toRadians(getSprocketAngle()),0);
        sprocketLeftMotor.set(-sprocketFeedforwardcalculation);
        sprocketRightMotor.set(sprocketFeedforwardcalculation);
    }
    /*
     * Ensures that the sprocket does not exceed 90 degrees
     */
    public void sprocketLimitStop(){
        if (getSprocketAngle()>90){
            stopSprocket();
        }

    }
    

}
