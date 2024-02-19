package frc.robot.subsystems.Vision;

import frc.robot.Constants;
import frc.lib.util.Limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PoseEstimator s_PoseEstimator;
    public Limelight LL;

    public Vision(PoseEstimator s_PoseEstimator) {
        this.s_PoseEstimator = s_PoseEstimator;
        LL = new Limelight(Constants.Vision.leftLLName);

    }

    public boolean HasTarget(){
        return (LL.getBlueBotPose2d().getX() != 0 && LL.getBlueBotPose2d().getY() != 0);
    }

   
    
    public void updateVision(){
        double x = LL.getBlueBotPose2d().getX();
        double y = LL.getBlueBotPose2d().getY();
        boolean target = (x != 0 && y != 0);
        if(target){
            Pose2d LLpose = new Pose2d(x, y, Rotation2d.fromDegrees(0.0));
            double LLlatency = (LL.getLatency());
            s_PoseEstimator.updateVision(LLpose, LLlatency);
        }       
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("LL, X", LL.getBlueBotPose2d().getX());
        SmartDashboard.putNumber("LL, Y", LL.getBlueBotPose2d().getY());
       
        Logger.recordOutput("LL Pose", LL.getBlueBotPose2d());
        
         if(s_PoseEstimator.readyToUpdateVision()){
            updateVision();
        } 
        
    }
}