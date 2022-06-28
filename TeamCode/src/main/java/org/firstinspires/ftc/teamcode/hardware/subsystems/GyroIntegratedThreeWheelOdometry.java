package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Odometry;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.function.DoubleSupplier;

public class GyroIntegratedThreeWheelOdometry extends Odometry {

    // Necessary Variables
    double trackWidth, ticksPerRevolution, wheelRadius, lateralOffset;
    inputMeasurement measurement = null;
    DoubleSupplier left = null, right = null, lateral = null;

    Pose2D position = new Pose2D(0, 0, Math.toRadians(0)), startPosition = new Pose2D(0, 0, Math.toRadians(0));

    public double accumulatedDistance = 0;

    boolean kalmanInit = false;
    double x = 0.0; // your initial state
    double Q = 1; // your model covariance
    double R = 3; // your sensor covariance
    double p = 1; // your initial covariance guess
    double K = 0.0; // your initial kalman gain guess

    double x_previous = x;
    double p_previous = p;

    public double kalmanOutput = 0;

    boolean hasInitRun = false;

    public IMU imu;

    public enum inputMeasurement {
        INCH,
        FEET,
        YARD,
        MM,
        CM,
        M,
    }

    public GyroIntegratedThreeWheelOdometry (DriveTrain driveTrain) {
        imu = new IMU(driveTrain.getHardwareMap());
    }

    public void initDoubleSuppliers(DoubleSupplier left, DoubleSupplier right, DoubleSupplier lateral) {
        this.left = left;
        this.right = right;
        this.lateral = lateral;
    }

    public void setConstants(double trackWidth, double ticksPerRevolution, double wheelRadius, double lateralOffset) {
        this.trackWidth = trackWidth;
        this.ticksPerRevolution = ticksPerRevolution;
        this.wheelRadius = wheelRadius;
        this.lateralOffset = lateralOffset;
    }

    public void setKalmanConstants(double x, double Q, double R, double p, double K) {
        kalmanInit = true;
        this.x = x;
        this.Q = Q;
        this.R = R;
        this.p = p;
        this.K = K;
    }

    public void setMeasurement(inputMeasurement measurement) {
        this.measurement = measurement;
    }

    public void setStartPosition(Pose2D startPosition) {
        this.position = startPosition;
    }

    /**
     * Take ticks converted to inches
     * @param ticks value of unadjusted tick readings
     * @return ticks converted to inches
     */

    private double convertTicksToInches(double ticks) {
        return ticks * (2.0 * Math.PI * wheelRadius / ticksPerRevolution);
    }

    /**
     * Take ticks converted to inches and change it to the specified measurement
     * @param ticks value of unadjusted tick readings
     * @return ticks converted to measurement
     */
    private double convertTicks(double ticks) {
        switch (measurement) {
            case INCH:
                ticks = convertTicksToInches(ticks);
                break;
            case FEET:
                ticks = convertTicksToInches(ticks) / 12.0;
                break;
            case YARD:
                ticks = convertTicksToInches(ticks) / 36.0;
                break;
            case MM:
                ticks = convertTicksToInches(ticks) * 25.4;
                break;
            case CM:
                ticks = convertTicksToInches(ticks) * 2.54;
                break;
            case M:
                ticks = convertTicksToInches(ticks) / 39.37;
                break;
        }
        return ticks;
    }

    private void correctAngle() {
        if(position.heading > Math.PI * 2.0) {
            position.heading -= 2.0 * Math.PI;
        }
        else if(position.heading < 0) {
            position.heading += 2.0 * Math.PI;
        }
    }

    @Override
    public void init() throws InterruptedException {
        if(trackWidth == 0.0) {
            throw new InterruptedException("trackWidth has no value");
        }

        if(ticksPerRevolution == 0.0) {
            throw new InterruptedException("ticksPerRevolution has no value");
        }

        if(wheelRadius == 0.0) {
            throw new InterruptedException("wheelRadius has no value");
        }

        if(measurement == null) {
            throw new InterruptedException("type must have a value (ie: INCH)");
        }

        if(left == null) {
            throw new InterruptedException("Left must have a double supplying it");
        }

        if(right == null) {
            throw new InterruptedException("Right must have a double supplying it");
        }

        if(lateral == null) {
            throw new InterruptedException("Lateral must have a double supplying it");
        }
        if(kalmanInit == false) {
            throw new InterruptedException("Set Values for the Kalman Gains");
        }

        imu.init();

        // Set Position to where we start
        position = startPosition;

        // End init
        hasInitRun = true;
    }

    double previousLeft = 0;
    double previousRight = 0;
    double previousLateral = 0;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void update() throws InterruptedException {
        if(!hasInitRun) {
            throw new InterruptedException("Something is wrong! Make sure to run the init()");
        }

        imu.update();

        double currentLeft = convertTicks(left.getAsDouble());
        double currentRight = convertTicks(right.getAsDouble());
        double currentLateral = convertTicks(lateral.getAsDouble());

        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;
        double dLateral = currentLateral - previousLateral;

        previousLeft = currentLeft;
        previousRight = currentRight;
        previousLateral = currentLateral;

        double dTheta = (dLeft - dRight) / (trackWidth);
        double dx = dLateral - lateralOffset * dTheta;
        double dy = (dLeft + dRight) / 2.0;

        double newAngle = position.heading + dTheta / 2;

        //Point deltaPosition = new Point(dy, dx).rotate(getKalmanHeading()).invertPoint();
        Point deltaPosition = new Point(dx, dy).rotate(newAngle);

        deltaPosition.invertX();

        position.addPoint(deltaPosition);
        position.heading += dTheta;

        accumulatedDistance += Math.hypot(deltaPosition.x, deltaPosition.y);

        correctAngle();
        mergeValues(dTheta);

        updateAcumulatedHeading();
    }

    public double previousHeading = 0;
    public double accumulatedHeading = 0;

    public void updateAcumulatedHeading() {
        double currentHeading = Math.toDegrees(getPose().getHeading());

        double dHeading = currentHeading - previousHeading;

        if(dHeading < -180) {
            dHeading += 360;
        }
        else if(dHeading >= 180) {
            dHeading -=360;
        }

        accumulatedHeading -= dHeading;
        previousHeading = currentHeading;
    }

    public void mergeValues(double deltaTheta) {
        x = x_previous + deltaTheta;

        p = p_previous + Q;

        K = p/(p + R);

        x += K * (imu.getHeadingInRadians() - x);

        p *= 1 - K;

        x_previous = x;
        p_previous = p;
    }

    public void resetAccumulatedDistance() {
        accumulatedDistance = 0;
    }

    public double getKalmanHeading() {
        return x;
    }

    public Pose2D getPose() {
        return position;
    }
}
