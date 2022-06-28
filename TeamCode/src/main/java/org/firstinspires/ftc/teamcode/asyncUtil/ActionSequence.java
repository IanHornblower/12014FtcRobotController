package org.firstinspires.ftc.teamcode.asyncUtil;

import org.firstinspires.ftc.teamcode.asyncUtil.actions.*;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

public class ActionSequence {

    ArrayList<Action> actionSequence;

    DriveTrain dt;

    public ActionSequence(RobotBase robot) {
        dt = robot.driveTrain;
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

    public void addEncoderDrive(double x, double y, double h, int ticks) {
        actionSequence.add(new EncoderDrive(dt, x, y, h, ticks));
    }

    public void addEncoderDrive(double x, double y, double h, int ticks, boolean fieldCentric) {
        actionSequence.add(new EncoderDrive(dt, x, y, h, ticks, fieldCentric));
    }

    public void addEncoderDriveEx(double x, double y, double h, int ticks) {
        actionSequence.add(new EncoderDriveEx(dt, x, y, h, ticks));
    }

    public void addEncoderDriveEx(double x, double y, double h, int ticks, boolean fieldCentric) {
        actionSequence.add(new EncoderDriveEx(dt, x, y, h, ticks, fieldCentric));
    }

    public void addFollowActualTrajectory(Trajectory traj) {
        actionSequence.add(new FollowActualTrajectory(dt, traj));
    }

    public void addFollowHolonomicPurePursuitTrajectory(Trajectory traj, double radius) {
        actionSequence.add(new FollowHolonomicPurePursuitTrajectory(dt, traj, radius));
    }

    public void addFollowPurePursuitTrajectory(Trajectory trajectory, double radius) {
        actionSequence.add(new FollowPurePursuitTrajectory(dt, trajectory, radius));
    }

    public void addFollowPurePursuitTrajectory(Trajectory trajectory, double radius, boolean reversed) {
        actionSequence.add(new FollowPurePursuitTrajectory(dt, trajectory, radius, reversed));
    }

    public void addForwardBackwardControl(int ticks) {
        actionSequence.add(new ForwardBackwardControl(dt, ticks));
    }

    public void addIMURotate(double heading) {
        actionSequence.add(new IMURotate(dt, heading));
    }

    public void addRotate(double heading) {
        actionSequence.add(new Rotate(dt, heading));
    }

    public void addRunToPosition(Pose2D pos) {
        actionSequence.add(new RunToPosition(dt, pos));
    }

    public void addWait(double seconds) {
        actionSequence.add(new Wait(seconds));
    }

}
