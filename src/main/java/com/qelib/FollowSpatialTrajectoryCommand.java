package com.qelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FollowSpatialTrajectoryCommand extends Command {
    private final SpatialTrajectory trajectory;
    private final Supplier<Pose2d> poseProvider;
    private final Consumer<ChassisSpeeds> outputSpeeds;
    private final SpatialSwerveFollower follower;
    private final Map<String, Command> eventMap;

    // How close the robot needs to be to a marker to trigger it
    private static final double MARKER_TOLERANCE_METERS = 0.2;

    public FollowSpatialTrajectoryCommand(
            SpatialTrajectory trajectory,
            Supplier<Pose2d> poseProvider,
            Consumer<ChassisSpeeds> outputSpeeds, // Expects ROBOT-CENTRIC ChassisSpeeds
            SpatialSwerveFollower follower,
            Map<String, Command> eventMap) {
        
        this.trajectory = trajectory;
        this.poseProvider = poseProvider;
        this.outputSpeeds = outputSpeeds;
        this.follower = follower;
        this.eventMap = eventMap;
    }

    @Override
    public void initialize() {
        follower.reset();
        // Reset all spatial markers so they can be fired again
        for (var marker : trajectory.markers) {
            marker.triggered = false;
        }
    }

    @Override
    public void execute() {
        Pose2d currentPose = poseProvider.get();

        // 1. Follow the path
        ChassisSpeeds speeds = follower.calculate(currentPose, trajectory);
        outputSpeeds.accept(speeds);

        // 2. Check spatial event markers
        for (var marker : trajectory.markers) {
            if (!marker.triggered) {
                double distanceToMarker = currentPose.getTranslation()
                        .getDistance(marker.triggerPose.getTranslation());

                // If robot's physical location gets close enough to the event's location
                if (distanceToMarker < MARKER_TOLERANCE_METERS) {
                    marker.triggered = true;
                    Command eventCmd = eventMap.get(marker.eventName);
                    if (eventCmd != null) {
                        eventCmd.schedule(); // Fire and forget the command
                    }
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished(poseProvider.get(), trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when finished
        outputSpeeds.accept(new ChassisSpeeds(0, 0, 0));
    }
}