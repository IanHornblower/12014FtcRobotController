package org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces;

public abstract class Odometry implements Subsystem {

    @Override
    public abstract void init() throws InterruptedException;

    @Override
    public abstract void update() throws InterruptedException;

}
