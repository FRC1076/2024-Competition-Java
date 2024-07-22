package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
public class Shooter extends SubsystemBase{
    public final CANSparkMax leftShootingMotor;
    public final CANSparkMax rightShootingMotor;
    
    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;     //Initialize
    PIDController leftShooterPID = new PIDController(0.0002, 0, 0);
    PIDController rightShooterPID = new PIDController(0.0003, 0.0001, 0.0001);

    
    public Shooter(){
        leftShootingMotor = new CANSparkMax(ShooterConstants.shooterLeftMotorID, CANSparkMax.MotorType.kBrushless);
        rightShootingMotor = new CANSparkMax(ShooterConstants.shooterRightMotorID, CANSparkMax.MotorType.kBrushless);
        leftShootingMotor.setInverted(ShooterConstants.shooterLeftInversion);  //Inverts the motor because one motor is on the other side
        rightShootingMotor.setInverted(ShooterConstants.shooterRightInversion);
        leftShooterEncoder = leftShootingMotor.getEncoder();
        rightShooterEncoder = rightShootingMotor.getEncoder();
        leftShootingMotor.enableVoltageCompensation(12);
        rightShootingMotor.enableVoltageCompensation(12);
        leftShootingMotor.setOpenLoopRampRate(ShooterConstants.shooterOpenLoopRampRate);
        rightShootingMotor.setOpenLoopRampRate(ShooterConstants.shooterOpenLoopRampRate);

         //declare
        
    }
    public double getLeftShooterRPM(){
        return leftShooterEncoder.getVelocity();
    }
    public double getRightShooterRPM(){
        return rightShooterEncoder.getVelocity();
    }
    public void setLeftShooterSpeed(double speed){
        leftShootingMotor.set(speed * ShooterConstants.RPMToPercentConversionFactor + leftShooterPID.calculate(leftShooterEncoder.getVelocity(),speed));
    }
    public void setRightShooterSpeed(double speed){
        rightShootingMotor.set(speed * ShooterConstants.RPMToPercentConversionFactor + rightShooterPID.calculate(rightShooterEncoder.getVelocity(),speed));
    }
    public void shootNote(){
        setLeftShooterSpeed(4500);
        setRightShooterSpeed(4500);
    }
    public void shootAmp(){
        setLeftShooterSpeed(2000);
        setRightShooterSpeed(2000);
    }
    public void shootReverse(){
        setLeftShooterSpeed(ShooterConstants.shooterLeftReverseSpeed);
        setRightShooterSpeed(ShooterConstants.shooterRighReverseSpeed);
    }
    

}
