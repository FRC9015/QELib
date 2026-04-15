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
        // Convert samples to Spatial States
        for (ChoreoTrajectoryDTO.Sample s : dto.samples) {
            states.add(new State(
                new Pose2d(s.x, s.y, Rotation2d.fromRadians(s.heading)),
                s.velocityX,
                s.velocityY,
                s.angularVelocity
            ));
        }

        // Convert time-based events to Spatial Markers
        if (dto.events != null) {
            for (ChoreoTrajectoryDTO.Event e : dto.events) {
                Pose2d poseAtEvent = getPoseAtTime(dto.samples, e.timestamp);
                markers.add(new SpatialMarker(poseAtEvent, e.name));
            }
        }
    }

    // Helper to find where the robot should be at a given time to place the spatial marker
    private Pose2d getPoseAtTime(ChoreoTrajectoryDTO.Sample[] samples, double time) {
        for (int i = 0; i < samples.length - 1; i++) {
            if (samples[i].timestamp <= time && samples[i+1].timestamp >= time) {
                // Snap to closest sample for simplicity (Can be linearly interpolated for higher precision)
                return new Pose2d(samples[i].x, samples[i].y, Rotation2d.fromRadians(samples[i].heading));
            }
        }
        // Fallback to last pose
        var last = samples[samples.length - 1];
        return new Pose2d(last.x, last.y, Rotation2d.fromRadians(last.heading));
    }
}