package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
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
    ArmFeedforward sprocketFeedforward = new ArmFeedforward(0, 0.0275, 0);
    PIDController pidController = new PIDController(Constants.armConstants.controllerGain, 0, 0);
    
    public Arm() {
        sprocketLeftMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
        sprocketRightMotor = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);
        sprocketLeftEncoder = sprocketLeftMotor.getEncoder();
        sprocketRightEncoder = sprocketRightMotor.getEncoder();
        sprocketLeftPIDController = sprocketLeftMotor.getPIDController();
        sprocketRightPIDController = sprocketRightMotor.getPIDController();
        sprocketLeftPIDController.setP(Constants.armConstants.controllerGain);
        sprocketRightPIDController.setP(Constants.armConstants.controllerGain); //declare
        
    }
    public void setSprocketSpeed(double speed) {
        sprocketLeftMotor.set(speed);
        sprocketRightMotor.set(speed);
    }
    public double getsprocketAngle() {
        return sprocketLeftEncoder.getPosition();
    }
    public double getSprocketSpeed(){
        return sprocketLeftEncoder.getVelocity();
    }
    public void sprocketfullspeeddown(){
        sprocketLeftMotor.set(-1);
        sprocketRightMotor.set(-1);
    }
    public void sprocketfullspeedup(){
        sprocketLeftMotor.set(1);
        sprocketRightMotor.set(1);
    }
    public boolean sprocketToPosition(double targetPos){
        double SprocketPIDCalculations = pidController.calculate(getsprocketAngle(), targetPos);
        double feedforwardcalculation = sprocketFeedforward.calculate(Math.toRadians(getsprocketAngle()),0.0);
        if(targetPos <= -30){
            SprocketPIDCalculations /= 2;
    }
            double output = SprocketPIDCalculations + feedforwardcalculation;
            sprocketLeftMotor.set(output);
            sprocketRightMotor.set(output);
    
        sprocketLimitStop();
        return Math.abs(targetPos - getsprocketAngle()) < 1;
    }
    public void sprocketLimitStop(){
        if (getsprocketAngle()>90){
            sprocketLeftMotor.stopMotor();
            sprocketRightMotor.stopMotor();
        }
    }

}
