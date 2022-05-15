package org.firstinspires.ftc.teamcode.asyncUtil;

public abstract class Action {

    protected boolean isComplete = false;

    public double error;

    public abstract void startAction();

    public abstract void runAction() throws InterruptedException;

    public abstract void stopAction();

    public boolean isActionComplete() {
        return isComplete;
    }

    public boolean isActionPersistent() {
        return false;
    }

    public boolean isAMultipleAction() {
        return false;
    }

}
