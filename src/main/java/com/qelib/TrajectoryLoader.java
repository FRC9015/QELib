package com.qelib;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;

public class TrajectoryLoader {
    private static final ObjectMapper mapper = new ObjectMapper();

    /**
     * Loads a Choreo .traj file from the deploy directory.
     * @param trajectoryName Name of file (e.g. "MyPath" for "deploy/choreo/MyPath.traj")
     * @return The parsed SpatialTrajectory
     */
    public static SpatialTrajectory loadChoreo(String trajectoryName) {
        File trajFile = new File(Filesystem.getDeployDirectory(), "choreo/" + trajectoryName + ".traj");
        
        try {
            ChoreoTrajectoryDTO dto = mapper.readValue(trajFile, ChoreoTrajectoryDTO.class);
            return new SpatialTrajectory(dto);
        } catch (IOException e) {
            System.err.println("Failed to load Choreo trajectory: " + trajectoryName);
            e.printStackTrace();
            return null;
        }
    }
    
    public static ArrayList<SpatialTrajectory> loadDeploy(){
        ArrayList<SpatialTrajectory>trajectories = new ArrayList<SpatialTrajectory>();
        FilenameFilter filter = new FilenameFilter() {
            @Override
            public boolean accept(File dir, String name) {
                return name.endsWith(".traj");
            }
        };
        for (File traj : Filesystem.getDeployDirectory().listFiles(filter)){
            trajectories.add(loadChoreo(traj.getName()));
        }
        return trajectories;
    }
}

