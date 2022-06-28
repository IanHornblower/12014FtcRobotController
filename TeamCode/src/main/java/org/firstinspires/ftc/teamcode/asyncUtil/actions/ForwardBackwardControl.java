package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class ForwardBackwardControl extends Action {

    BasicPID fwController = new BasicPID(DriveConstants.forwardBackwardPID);

    DriveTrain dt;
    double totalTicks;
    int ticks;

    public ForwardBackwardControl(DriveTrain dt, int ticks) {
        this.dt = dt;
        this.ticks = ticks;
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        totalTicks = (dt.motors[0].getCurrentPosition() + dt.motors[1].getCurrentPosition() + dt.motors[2].getCurrentPosition() + dt.motors[3].getCurrentPosition())/4.0;

        fwController.calculate(ticks, totalTicks);

        dt.setMotorPowers(fwController.calculate(ticks, totalTicks), fwController.calculate(ticks, totalTicks), fwController.calculate(ticks, totalTicks), fwController.calculate(ticks, totalTicks));

        isComplete = Math.abs(totalTicks) > Math.abs(ticks) && dt.getCombinedVelocity() < 0.5;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
