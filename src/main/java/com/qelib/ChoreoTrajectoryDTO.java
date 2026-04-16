package com.qelib;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

@JsonIgnoreProperties(ignoreUnknown = true)
public class ChoreoTrajectoryDTO {
    public String name;
    public int version;
    
    // Jackson will look for the "trajectory" block in the JSON
    @JsonProperty("trajectory")
    public TrajectoryData trajectory;

    // Events are at the top level in your JSON
    @JsonProperty("events")
    public Event[] events = new Event[0];

    // --- NESTED CLASSES ---

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class TrajectoryData {
        @JsonProperty("samples")
        public Sample[] samples = new Sample[0];
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Sample {
        @JsonProperty("t") public double timestamp;
        @JsonProperty("x") public double x;
        @JsonProperty("y") public double y;
        @JsonProperty("heading") public double heading;
        @JsonProperty("vx") public double velocityX;
        @JsonProperty("vy") public double velocityY;
        @JsonProperty("omega") public double angularVelocity;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Event {
        @JsonProperty("t") public double timestamp;
        @JsonProperty("name") public String name;
    }
}