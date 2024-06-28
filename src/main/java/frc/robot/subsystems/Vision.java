package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase{

    public String limeString;
    private double tx, ty, ta;
    private int aprilTags;
    private int retroReflective;
    private double minTargetAspectRationReflective, maxTargetAspectRationReflective;
    private double minTargetAspectRationAprilTag, maxTargetAspectRationAprilTag;
    private boolean shouldUpdatePose;

    private static final double conversionScalar = 39.37;
    private static final String defaultLimeString = "limelight";

    public Vision(String _limelightString) {
        this.limeString = _limelightString;
    }

    public Vision() {this.limeString = defaultLimeString;}
    
    @Override
    public void periodic(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable(this.limeString);
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        ta = table.getEntry("ta").getDouble(0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
    }

    public double[] getPose(){
        return NetworkTableInstance.getDefault().getTable(limeString).getEntry("botpose").getDoubleArray(new double[6]);
    }

    public double getTargetX(){
        return tx;
    }

    public double getTargetY(){
        return ty;
    }

    public double getTargetArea(){
        return ta;
    }


}
