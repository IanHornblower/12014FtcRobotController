package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;

public class Duck implements Subsystem {

    HardwareMap hwMap;

    public Duck(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }

    public void resetEncoder() {

    }

    @Override
    public void init() throws InterruptedException {

    }

    @Override
    public void update() throws InterruptedException {

    }
}
