package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftMillimeters, rightMillimeters, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "SkyStoneAutonomousRed", group = "OpMode")
//@Disabled
public class Skystone_Autonomus_Red extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTest robot = new HardwareTest();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CENTIMETERS = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CENTIMETER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTIMETERS * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.8;
    static final double DIST_PER_REV = (4 * 2.54 * Math.PI) / 1120;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        telemetry.addData("ServoPos", robot.armServo.getPosition());
        telemetry.addData("ServoPosGrip", robot.gripperServo.getPosition());
        telemetry.update();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        stoneDetection();
        gyroStrafe(-1.571,0.0, 30, 0.5,5.0);
        gyroStrafe(0.0,-90.0,165,0.6,10.0);
        robot.gripperServo.setPosition(0.1);
        waitMilis(100);
        gyroStrafe(-3.1416,-90,60,0.8,5.0);
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void resetEncoders () {
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    // 0.25 MAX & 1.0 MIN
    public void arm (double armPos,
                     double gripperPos,
                     double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {

            robot.gripperServo.setPosition(gripperPos);
            robot.armServo.setPosition(armPos);
        }
    }

    public void gyroDrive (double heading,
                           double distanceCM,
                           double power,
                           double timeoutS) {
        resetEncoders();
        double currentHeading;
        double drvPower = power;
        double adjPower;
        double currentDistance;
        double distanceTicks;
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            adjPower = (currentHeading - heading) / 50;
            currentDistance = robot.leftDrive.getCurrentPosition();
            distanceTicks = (distanceCM * COUNTS_PER_CENTIMETER);
            if (currentDistance > distanceTicks) {
                robot.leftDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                return;
            }
            robot.leftDrive.setPower(drvPower - adjPower);
            robot.leftBackDrive.setPower(drvPower - adjPower);
            robot.rightDrive.setPower(drvPower + adjPower);
            robot.rightBackDrive.setPower(drvPower + adjPower);

        }
    }

    public void gyroTurn (double heading,
                          double timeoutS) {
        double currentHeading;
        double power;
//        double p = 0;
//        double i = 0;
//        double d = 0;
//        double pK = .05;
//        double iK = .5;
//        double dK = .05;
//        double oldHeading = 0;
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {
            currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//            p = (currentHeading - heading) * pK;
//            i = p * iK + i;
//            d = (oldHeading - currentHeading) * dK;
//            power = p+i+d;
            power = (currentHeading - heading) / 50;
            if (power < 0) {
                power = -Math.max( 0.1, Math.abs(power));
            }
            else{
                power = Math.max(0.1, Math.abs(power));
            }
            if (Math.abs(currentHeading - heading) < 2.0) {
                return;
            }

            robot.leftDrive.setPower(-power);
            robot.leftBackDrive.setPower(-power);
            robot.rightDrive.setPower(power);
            robot.rightBackDrive.setPower(power);
            telemetry.addData("Gyro","Running");
        }
    }
    public double subtractAngle(double angleA,
                                double angleB){
        double result;
        result = angleA - angleB;
        if (result > 180){
            result = result - 360;
        }
        return result;
    }
    public void gyroStrafe (double heading,
                            double pose,
                            double distance,
                            double power,
                            double timeoutS) {
        resetEncoders();
        double currentHeading;
        double drvPower = power;
        double adjPower;
        double currentDistance;
        double driveAngle;
        double headingRadians;
        double distX = 0;
        double distY = 0;
        double dY;
        double dX;
        double oldLeft = 0;
        double oldRight = 0;
        double oldBackLeft = 0;
        double oldBackRight = 0;
        double newLeft = 0;
        double newRight = 0;
        double newBackLeft = 0;
        double newBackRight = 0;
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            headingRadians = ((-currentHeading/180)*3.1416)+(1/2*3.1416);
            driveAngle = subtractAngle(pose,currentHeading);
            adjPower = subtractAngle(pose,currentHeading) / 80;
            newLeft = robot.leftDrive.getCurrentPosition();
            newRight = robot.rightDrive.getCurrentPosition();
            newBackRight = robot.rightBackDrive.getCurrentPosition();
            newBackLeft = robot.leftBackDrive.getCurrentPosition();

            dX = ((((newLeft - oldLeft) - (newBackLeft - oldBackLeft)
                    - (newRight - oldRight) + (newBackRight - oldBackRight)) * Math.sin(Math.PI/4)) / 4.0) * DIST_PER_REV;
            dY = (((newLeft - oldLeft) + (newBackLeft - oldBackLeft)
                    + (newRight - oldRight) + (newBackRight - oldBackRight)) / 4.0) * DIST_PER_REV;
            distX += (Math.sin(headingRadians) * dX + Math.cos(headingRadians) * dY);
            distY += (Math.sin(headingRadians) * dY + Math.cos(headingRadians) * dX);
            oldLeft = newLeft;
            oldRight = newRight;
            oldBackLeft = newBackLeft;
            oldBackRight = newBackRight;

            currentDistance = Math.sqrt(distX * distX + distY * distY);
            if (currentDistance > distance) {
                robot.leftDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                return;
            }

            driveAngle = ((-currentHeading/180)*3.1416)+(1/2*3.1416)+heading;
//            driveAngle = 0.5*Math.PI;
//            adjPower = 0;

            double robotAngle = driveAngle - Math.PI / 4;
            final double v1 = power * Math.cos(robotAngle) - adjPower;
            final double v2 = power * Math.sin(robotAngle) + adjPower;
            final double v3 = power * Math.sin(robotAngle) - adjPower;
            final double v4 = power * Math.cos(robotAngle) + adjPower;

            telemetry.addData("gyro angle", currentHeading);
            telemetry.update();

            robot.leftDrive.setPower(v1);
            robot.rightDrive.setPower(v2);
            robot.leftBackDrive.setPower(v3);
            robot.rightBackDrive.setPower(v4);
        }
    }
    public void encoderDrive(double speed,
                             double leftCentimeters, double rightCentimeters,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftCentimeters * COUNTS_PER_CENTIMETER);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightCentimeters * COUNTS_PER_CENTIMETER);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.leftBackDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void waitMilis (double timeOutMs) {

        runtime.reset();
        while (runtime.milliseconds() < timeOutMs);
    }
    public void stoneDetection () {
        double timeoutS = 7.0;
        double hueValue = 0.4;
        double redU = 0.0;
        double greenU = 0.0;
        double blueU = 0.0;
        boolean foundPos = false;
        int run = 0;
        runtime.reset();
        gyroStrafe(3.1416,0,50,0.5,400.0);
        gyroStrafe(1.571,0.0,60,0.5, 20.0);
        while (opModeIsActive() && (!foundPos) && (timeoutS > runtime.seconds())) {
            redU = (double) robot.colorSensorRed.red() / (double) robot.colorSensorRed.alpha();
            greenU = (double) robot.colorSensorRed.green() / (double) robot.colorSensorRed.alpha();
            blueU = (double) robot.colorSensorRed.blue() / (double) robot.colorSensorRed.alpha();

            if (((greenU + redU) * hueValue) < blueU) {
                //Sees skystone
                telemetry.addData("Skystone Scrunchied", "Retreving");
                telemetry.update();
                //strafe to pos?
                robot.armServo.setPosition(0.9);
               // gyroDrive(0.0,4,0.5,0.5);
                gyroStrafe(2.356, 0, 5.657, 0.5, 20.0);
                robot.gripperServo.setPosition(1.0);
                waitMilis(600);
                robot.armServo.setPosition(0.6);
                foundPos = true;

                return;
            }
            else {
                gyroStrafe(0.0,0.0,7,0.5,100000.0);
                run++;
                telemetry.addData("NoSkystone", "moving");
                telemetry.update();
            }
        }
    }
}

