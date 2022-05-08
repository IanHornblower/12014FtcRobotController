package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;

public class IMU implements Subsystem {

    BNO055IMU imu;
    HardwareMap hwMap;
    Orientation angles;

    AxesOrder HubConfig = AxesOrder.ZYX;

    double previousHeading = 0.0;
    double accumulatedHeading = 0.0;

    public IMU(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }

    @Override
    public void init() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;  // :)
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void update() throws InterruptedException {
        double currentHeading = getHeadingInRadians();
        double dHeading = currentHeading - previousHeading;

        if(dHeading < -180) {
            dHeading += 360;
        }
        else if(dHeading >= 180) {
            dHeading -=360;
        }

        accumulatedHeading += dHeading;
        previousHeading = currentHeading;
    }

    public double getRawHeading() {
        double currentHeading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, HubConfig, AngleUnit.RADIANS);
        currentHeading = -angles.firstAngle;
        return currentHeading;
    }



    public double getHeadingInRadians() {
        double adjustedHeading = getRawHeading();

        if(adjustedHeading < 0.0) {
            adjustedHeading += Math.PI * 2.0;
        }

        return adjustedHeading;
    }

    public double getHeadingInDegrees() {
        return Math.toDegrees(getRawHeading());
    }

    public double getAccumulatedHeadingInRadians() {
        return accumulatedHeading;
    }

    public double getAccumulatedHeadingInDegrees() {
        return Math.toDegrees(accumulatedHeading);
    }

    public void resetHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, HubConfig, AngleUnit.RADIANS);
        angles.firstAngle = 0;
    }
}
