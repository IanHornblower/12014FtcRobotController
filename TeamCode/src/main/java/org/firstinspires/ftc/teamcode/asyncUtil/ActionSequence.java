package org.firstinspires.ftc.teamcode.asyncUtil;

import org.firstinspires.ftc.teamcode.asyncUtil.actions.*;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

public class ActionSequence {

    ArrayList<Action> actionSequence;

    public ActionSequence() {
        actionSequence = new ArrayList<>();
    }

    public ActionSequence(ArrayList<Action> actionSequence) {
        this.actionSequence = actionSequence;
    }

    public ArrayList<Action> getActionList() {
        return actionSequence;
    }

    public void addAction(Action action) {
        actionSequence.add(action);
    }

    public void addCustomAction(Runnable runnable) {
        actionSequence.add(new CustomAction(runnable));
    }

    public void addRotate(DriveTrain dt, double heading) {
        actionSequence.add(new Rotate(dt, heading));
    }

    public void addRunToPosition(DriveTrain dt, Pose2D pos) {
        actionSequence.add(new RunToPosition(dt, pos));
    }

    public void addFollowSetHeadingHolonomicTrajectory(DriveTrain dt, Trajectory trajectory, double radius, double heading) {
        actionSequence.add(new FollowSetHeadingHolonomicTrajectory(dt, trajectory, radius, heading));
    }

    public void addFollowPurePursuitTrajectory(DriveTrain dt, Trajectory trajectory, double radius) {
        actionSequence.add(new FollowPurePursuitTrajectory(dt, trajectory, radius));
    }

    public void addFollowPurePursuitTrajectory(DriveTrain dt, Trajectory trajectory, double radius, boolean reversed) {
        actionSequence.add(new FollowPurePursuitTrajectory(dt, trajectory, radius, reversed));
    }
}
