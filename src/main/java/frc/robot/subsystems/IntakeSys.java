package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

    /**
     * Constructs a new ExampleSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */

    public IntakeSys() {
        //which constants are we using
        //how to create motor on java
    
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        //is this needed for intake?
    }

    public void intake() {
        //spin motor positive
    }

    public void eject() {
        //spin motor negative
    }

}