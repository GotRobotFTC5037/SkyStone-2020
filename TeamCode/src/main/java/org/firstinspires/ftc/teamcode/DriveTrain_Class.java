package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gyroscope;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.DriveTrain_Class;
//import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

//@Disabled
//public class DriveTrain_Class {
//BNO055IMU imu;
    //    private DcMotor bl;
//    private DcMotor br;
//    private DcMotor fl;

    //    private DcMotor fr;
//    Orientation angles;
//    Acceleration gravity;
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    public void run() {
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//    bl = hardwareMap.get(DcMotor.class, "bl");
//    br = hardwareMap.get(DcMotor.class, "br");
//    fl = hardwareMap.get(DcMotor.class, "fl");
//    fr = hardwareMap.get(DcMotor.class, "fr");
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//    parameters.loggingEnabled = true;
//    parameters.loggingTag = "IMU";
//    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        Orientation currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double distanceDriven = fl.getCurrentPosition();
//        double steering = 0.0;
//        double motorPower = 0.0;
//        Orientation headingDriveLeft;
//        ( double desiredHeading, double power, double desiredDistance);
//        {
//
//            if ((desiredDistance >= distanceDriven) || !(Math.abs(desiredHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle()) <= 2))
//                ;
//            {
//                steering = desiredHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle();
//                motorPower = power + steering;
//            }
//            return motorPower;
//        }
//
//        Orientation headingDriveRight (double desiredHeading, double power, double desiredDistance)
//        ;
//        {
//            if ((desiredDistance >= distanceDriven) || !(Math.abs(desiredHeading - currentHeading) <= 2))
//                ;
//            {
//                steering = desiredHeading - currentHeading;
//
//            }
//            return power - steering;
//        }
//
//    }
//
//}
//
