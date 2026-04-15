package com.qelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SpatialSwerveFollower {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    
    private int closestIndex = 0;
    private final double completionToleranceMeters;

    public SpatialSwerveFollower(PIDController xCtrl, PIDController yCtrl, PIDController thetaCtrl, double completionToleranceMeters) {
        this.xController = xCtrl;
        this.yController = yCtrl;
        this.thetaController = thetaCtrl;
        this.completionToleranceMeters = completionToleranceMeters;
        
        // Ensure rotational PID takes the shortest path
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        closestIndex = 0;
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    public ChassisSpeeds calculate(Pose2d currentPose, SpatialTrajectory trajectory) {
        if (trajectory.states.isEmpty()) return new ChassisSpeeds();

    // 1. Find the closest state
    int searchWindow = 25; 
    double minDistance = Double.MAX_VALUE;
    int maxSearch = Math.min(trajectory.states.size(), closestIndex + searchWindow);

    for (int i = closestIndex; i < maxSearch; i++) {
        double dist = currentPose.getTranslation().getDistance(trajectory.states.get(i).pose.getTranslation());
        if (dist < minDistance) {
            minDistance = dist;
            closestIndex = i;
        }
    }

    // --- OPTIMIZATION: PURE PURSUIT LOOKAHEAD ---
    // Look ahead 8 states (~0.16 seconds) to smooth out steering and maintain high speeds.
    int lookaheadStates = 8; 
    int targetIndex = Math.min(closestIndex + lookaheadStates, trajectory.states.size() - 1);
    
    // We get the Feedforward from the CLOSEST state (because that's what the robot should be doing right now)
    SpatialTrajectory.State closestState = trajectory.states.get(closestIndex);
    
    // We get the Feedback (PID target) from the LOOKAHEAD state (to pull the robot smoothly forward)
    SpatialTrajectory.State targetState = trajectory.states.get(targetIndex);

    // 2. Feedforward (Field Relative) - from the closest point
    double ffVx = closestState.velocityX;
    double ffVy = closestState.velocityY;
    double ffOmega = closestState.angularVelocity;

    // 3. Feedback (Correcting error) - pulling towards the lookahead point
    double fbVx = xController.calculate(currentPose.getX(), targetState.pose.getX());
    double fbVy = yController.calculate(currentPose.getY(), targetState.pose.getY());
    double fbOmega = thetaController.calculate(currentPose.getRotation().getRadians(), targetState.pose.getRotation().getRadians());

    // Combine and return
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        ffVx + fbVx, ffVy + fbVy, ffOmega + fbOmega, currentPose.getRotation()
        );
    }
    
    public boolean isFinished(Pose2d currentPose, SpatialTrajectory trajectory) {
        // We are done if we are at the end of the list AND physically close to the final point
        if (closestIndex >= trajectory.states.size() - 2) {
            Pose2d finalPose = trajectory.states.get(trajectory.states.size() - 1).pose;
            return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < completionToleranceMeters;
        }
        return false;
    }
}