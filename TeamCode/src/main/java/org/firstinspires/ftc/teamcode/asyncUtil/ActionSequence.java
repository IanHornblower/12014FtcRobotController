package org.firstinspires.ftc.teamcode.asyncUtil;

import org.firstinspires.ftc.teamcode.asyncUtil.actions.*;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;

import java.util.ArrayList;

public class ActionSequence {

    ArrayList<Action> actionSequence;

    public ActionSequence(RobotBase robot) {
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

    public void addWait(double seconds) {
        actionSequence.add(new Wait(seconds));
    }
}
