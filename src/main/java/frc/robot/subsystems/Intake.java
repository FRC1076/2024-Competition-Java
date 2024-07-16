package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    // Declare actuators, sensors, and other variables here
    public final CANSparkMax m_intakeMotor; 
    /**
     * Constructs a new ExampleSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public Intake() {
        // Initialize and configure actuators and sensors here
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, CANSparkMax.MotorType.kBrushless);
    }

    public void setMotorSpeed(double speed) {
        m_intakeMotor.set(speed);   
    }


    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}