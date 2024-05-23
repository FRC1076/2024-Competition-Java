package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

public class JoystickDrive extends Command {

    private final DriveSubsystem driveSubsystem;
    private DoubleSupplier fwd;
    private DoubleSupplier strafe;
    private DoubleSupplier rot;

    /**
     * Constructs a new JoystickDrive command
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param fwd The forward/backward value of the robot (positive indicates forward) (-1 to 1)
     * @param strafe The side to side value of the robot (-1 to 1)
     * @param rot The rotational value of the robot (-1 to 1)
     * @param driveSubsystem The required DriveSubsystem
     */
    public JoystickDrive(
        DoubleSupplier fwd, 
        DoubleSupplier strafe, 
        DoubleSupplier rot, 
        DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;
        this.fwd = fwd;
        this.strafe = strafe;
        this.rot = rot;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Scale the joystick values by the max speed of the robot (intended for closed loop control)
        driveSubsystem.drive(
            fwd.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond,
            strafe.getAsDouble()  * DriveConstants.kMaxSpeedMetersPerSecond,
            rot.getAsDouble()  * DriveConstants.kMaxRotationalSpeedRadiansPerSecond,
            true
        );
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}