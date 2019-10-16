package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



public class DriveTrain_Class {
    BNO055IMU imu;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor fr;
    Orientation angles;
    Acceleration gravity;
    double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    double distanceDriven = fl.getCurrentPosition();
    double steering;
    public double headingDriveLeft(double desiredHeading, double power, double desiredDistance) {
        if ((desiredDistance >= distanceDriven)||!(Math.abs(desiredHeading - currentHeading)<= 2)); {
            steering = desiredHeading - currentHeading;

        }
        return power + steering;
    }
    public double headingDriveRight(double desiredHeading, double power, double desiredDistance) {
        if ((desiredDistance >= distanceDriven)||!(Math.abs(desiredHeading - currentHeading)<= 2)); {
            steering = desiredHeading - currentHeading;

        }
        return power - steering;
    }
}
