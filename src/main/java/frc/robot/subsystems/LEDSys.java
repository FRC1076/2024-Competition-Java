package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDSys extends SubsystemBase{

    // Declare actuators, sensors, and other variables here
    public final DigitalOutput digitalPin7 = new DigitalOutput(7);
    public final DigitalOutput digitalPin8 = new DigitalOutput(8);
    public final DigitalOutput digitalPin9 = new DigitalOutput(9);



    /**
     * Constructs a new ExampleSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public LEDSys() {
        // Initialize and configure actuators and sensors here
    
    }

    // These colors are different than the ones in the Python code!!!

    public void setLEDPins(String color) {
        if(color == "purple"){
            // default color, nothing happening (1)
            digitalPin7.set(true);
            digitalPin8.set(false);
            digitalPin9.set(false);
        } else if(color == "purple-flash"){
            // note intaken (2)
            digitalPin7.set(false);
            digitalPin8.set(true);
            digitalPin9.set(false);
        } else if(color == "blue"){
            // note detected on the right (3)
            digitalPin7.set(true);
            digitalPin8.set(true);
            digitalPin9.set(false);
        } else if(color == "pink"){
            // note detected to the left (4)
            digitalPin7.set(false);
            digitalPin8.set(false);
            digitalPin9.set(true);
        } else if(color == "yellow-flash"){
            // note detected center (5)
            digitalPin7.set(true);
            digitalPin8.set(false);
            digitalPin9.set(true);
        } else if(color == "green-flash"){
            // speaker AprilTag detected (6)
            digitalPin7.set(false);
            digitalPin8.set(true);
            digitalPin9.set(true);
        } else if(color == "off"){
            // turn the LEDs off
            digitalPin7.set(false);
            digitalPin8.set(false);
            digitalPin9.set(false);
        }
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}