package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Odometry extends Thread {


    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private Hardware robot;
    double oldLeft;
    double oldRight;
    double oldBackLeft;
    double oldBackRight;

    Odometry(Hardware robot, BNO055IMU imu) {
        this.robot = robot;
        this.imu = imu;
        this.oldLeft = robot.leftDrive.getCurrentPosition();
        this.oldRight = robot.rightDrive.getCurrentPosition();
        this.oldBackLeft = robot.leftBackDrive.getCurrentPosition();
        this.oldBackRight = robot.rightBackDrive.getCurrentPosition();
    }
    public void waitMillis(double timeOutMs) {
        timeOutMs += runtime.milliseconds();
        while (runtime.milliseconds() < timeOutMs);
    }

    public void run() {
        try {

            while (!isInterrupted()) {
                getVelocity();
                //waitMillis(20);
                Thread.sleep(10);
            }
            return;
        }
        // catch (InterruptedException e) {System.out.println(e);}
        catch (Exception e) {System.out.println(e.toString());}
    }

    public void runOpMode() throws InterruptedException {
    }
    //Odometry odometry = null;

    //init for getVelocity
    double oldTime = runtime.seconds();
    double currentTime;
    double currentVelocity;
    double currentHeading;
    double currentDistance = 0;
    double headingRadians;
    double distX = 0;
    double distY = 0;
    double dY;
    double dX;
//    double currentttHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//    double oldLeft = robot.leftDrive.getCurrentPosition();
//    double oldRight = robot.rightDrive.getCurrentPosition();
//    double oldBackLeft = robot.leftBackDrive.getCurrentPosition();
//    double oldBackRight = robot.rightBackDrive.getCurrentPosition();
    double newLeft = 0;
    double newRight = 0;
    double newBackLeft = 0;
    double newBackRight = 0;

    //Get Velocity running
    public void getVelocity() {
        currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        headingRadians = ((-currentHeading / 180) * 3.1416) + (1 / 2 * 3.1416);
        newLeft = robot.leftDrive.getCurrentPosition();
        newRight = robot.rightDrive.getCurrentPosition();
        newBackRight = robot.rightBackDrive.getCurrentPosition();
        newBackLeft = robot.leftBackDrive.getCurrentPosition();

        dX = ((((newLeft - oldLeft) - (newBackLeft - oldBackLeft)
                - (newRight - oldRight) + (newBackRight - oldBackRight)) * Math.sin(Math.PI / 4)) / 4.0) * robot.DIST_PER_REV;
        dY = (((newLeft - oldLeft) + (newBackLeft - oldBackLeft)
                + (newRight - oldRight) + (newBackRight - oldBackRight)) / 4.0) * robot.DIST_PER_REV;
        distX += (Math.sin(headingRadians) * dX + Math.cos(headingRadians) * dY);
        distY += (Math.sin(headingRadians) * dY + Math.cos(headingRadians) * dX);
        oldLeft = newLeft;
        oldRight = newRight;
        oldBackLeft = newBackLeft;
        oldBackRight = newBackRight;

        currentDistance = Math.sqrt(distX * distX + distY * distY);
        currentVelocity = (currentDistance / (runtime.seconds() - oldTime));
        currentTime = runtime.seconds();
    }
}
