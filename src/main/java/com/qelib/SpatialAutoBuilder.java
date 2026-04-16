package com.qelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.controller.PIDController;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SpatialAutoBuilder {
    private static Supplier<Pose2d> poseProvider;
    private static Consumer<ChassisSpeeds> outputSpeeds;
    public static Map<String, Command> globalEventMap;

    // Configured PID values
    private static double kX, kY, kTheta;
    
    public static void configure(
            Supplier<Pose2d> poseProvider,
            Consumer<ChassisSpeeds> outputSpeeds,
            Map<String, Command> globalEventMap,
            double kX, double kY, double kTheta) {
        
        SpatialAutoBuilder.poseProvider = poseProvider;
        SpatialAutoBuilder.outputSpeeds = outputSpeeds;
        SpatialAutoBuilder.globalEventMap = globalEventMap;
        SpatialAutoBuilder.kX = kX;
        SpatialAutoBuilder.kY = kY;
        SpatialAutoBuilder.kTheta = kTheta;
    }

    public static void configurePID(
            double kX, double kY, double kTheta) {
        SpatialAutoBuilder.kX = kX;
        SpatialAutoBuilder.kY = kY;
        SpatialAutoBuilder.kTheta = kTheta;
    }

    public static void addEvent(String name, Command command){
        SpatialAutoBuilder.globalEventMap.put(name, command);
    }

    // Creates a single path command dynamically
    public static Command buildPath(String pathName) {
        SpatialTrajectory traj = TrajectoryLoader.loadChoreo(pathName);
        SpatialSwerveFollower follower = new SpatialSwerveFollower(
            new PIDController(kX, 0, 0),
            new PIDController(kY, 0, 0),
            new PIDController(kTheta, 0, 0),
            0.1
        );

        return new FollowSpatialTrajectoryCommand(traj, poseProvider, outputSpeeds, follower, globalEventMap);
    } 
}