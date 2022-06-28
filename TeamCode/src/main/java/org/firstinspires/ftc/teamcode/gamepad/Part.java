package org.firstinspires.ftc.teamcode.gamepad;

import org.firstinspires.ftc.teamcode.util.Timer;

public abstract class Part {

    public final double toggleDelay = 0.2;

    public Timer toggleTimer = new Timer();

    public abstract void init();

    public abstract void update();
}
