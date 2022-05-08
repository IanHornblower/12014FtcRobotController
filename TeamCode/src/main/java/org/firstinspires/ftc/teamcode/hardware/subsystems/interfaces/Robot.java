package org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces;

public interface Robot {

    void initHardwareMap() throws Exception;

    void update() throws Exception;

    void resetEncoders();

    void emergencyStop();

}
