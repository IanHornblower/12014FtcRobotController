package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.ArrayList;

public class TrajectorySequence {

    ArrayList<Trajectory> sequence;

    int completedCount = 0;
    boolean isCompleted = false;

    public TrajectorySequence(ArrayList<Trajectory> sequence) {
        this.sequence = sequence;
    }

    public boolean isCompleted() {
        return isCompleted;
    }

    public int getCompletedCount() {
        return completedCount;
    }

    public void update() {
        for(int i = 0; i < sequence.size(); i++) {
            if(sequence.get(i).isComplete()) completedCount++;
        }

        if(completedCount == sequence.size()) {
            isCompleted = true;
        }
    }


}
