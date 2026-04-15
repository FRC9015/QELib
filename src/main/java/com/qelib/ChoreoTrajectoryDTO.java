package com.qelib;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

@JsonIgnoreProperties(ignoreUnknown = true)
public class ChoreoTrajectoryDTO {
    public String name;
    public int version;
    public Sample[] samples;
    public Event[] events;

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Sample {
        public double timestamp;
        public double x;
        public double y;
        public double heading;
        public double velocityX;
        public double velocityY;
        public double angularVelocity;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Event {
        public double timestamp;
        public String name;
    }
}