package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SubsystemBase {

    // Declare actuators, sensors, and other variables here
    private final CANSparkMax m_indexMotor;
    private final DigitalInput m_beamBreak;
    /**
     * Constructs a new Indexer.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public Indexer() {
        // Initialize and configure actuators and sensors here
        m_indexMotor = new CANSparkMax(IndexerConstants.kIndexMotorPort, MotorType.kBrushless);
        m_beamBreak = new DigitalInput(IndexerConstants.kBeamBreakPin);
    }

    public void setIndexMotor(double speed) {
        m_indexMotor.set(speed);
    }

    public boolean getBeamBroken() {
        return !m_beamBreak.get();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}