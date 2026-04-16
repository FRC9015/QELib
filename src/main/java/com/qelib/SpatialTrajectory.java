package com.qelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class SpatialTrajectory {
    public final List<State> states = new ArrayList<>();
    public final List<SpatialMarker> markers = new ArrayList<>();

    public static class State {
        public final Pose2d pose;
        public final double velocityX;
        public final double velocityY;
        public final double angularVelocity;

        public State(Pose2d pose, double vx, double vy, double omega) {
            this.pose = pose;
            this.velocityX = vx;
            this.velocityY = vy;
            this.angularVelocity = omega;
        }
    }

    public static class SpatialMarker {
        public final Pose2d triggerPose;
        public final String eventName;
        public boolean triggered = false;

        public SpatialMarker(Pose2d triggerPose, String eventName) {
            this.triggerPose = triggerPose;
            this.eventName = eventName;
        }
    }

    public SpatialTrajectory(ChoreoTrajectoryDTO dto) {
        // Updated Safety Check: Check if dto, trajectory, or samples are null
        if (dto == null || dto.trajectory == null || dto.trajectory.samples == null || dto.trajectory.samples.length == 0) {
            System.err.println("CRITICAL ERROR: Trajectory was empty or failed to parse!");
            return;
        }

        // We now pull from dto.trajectory.samples
        for (ChoreoTrajectoryDTO.Sample s : dto.trajectory.samples) {
            states.add(new State(
                new Pose2d(s.x, s.y, edu.wpi.first.math.geometry.Rotation2d.fromRadians(s.heading)),
                s.velocityX,
                s.velocityY,
                s.angularVelocity
            ));
        }

        // Convert events into spatial markers
        if (dto.events != null) {
            for (ChoreoTrajectoryDTO.Event e : dto.events) {
                Pose2d poseAtEvent = getPoseAtTime(dto.trajectory.samples, e.timestamp);
                markers.add(new SpatialMarker(poseAtEvent, e.name));
            }
        }
    }

    // Helper method: Make sure to update the parameter type here too!
    private Pose2d getPoseAtTime(ChoreoTrajectoryDTO.Sample[] samples, double time) {
        for (int i = 0; i < samples.length - 1; i++) {
            if (samples[i].timestamp <= time && samples[i+1].timestamp >= time) {
                return new Pose2d(samples[i].x, samples[i].y, edu.wpi.first.math.geometry.Rotation2d.fromRadians(samples[i].heading));
            }
        }
        var last = samples[samples.length - 1];
        return new Pose2d(last.x, last.y, edu.wpi.first.math.geometry.Rotation2d.fromRadians(last.heading));
    }
}