/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@Autonomous(name = "Autonomous", group = "OpMode")
//@Disabled
public class Autonomous_Depot extends LinearOpMode {

    /* Declare OpMode members. */
    BNO055IMU imu;
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
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
    private Error markerError;

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Functions fun = new Functions(robot, imu);

        fun.waitMilis(50);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        if (robot.leftMarkerSwitch.getVoltage() > 3.0 && robot.rightMarkerSwitch.getVoltage() > 3.0) {
            telemetry.addData("Running Blue", "Waiting For Start");
            telemetry.update();
            waitForStart();
            fun.gyroStrafe(1.57, 1.57, 5, 1, 5);
            fun.resetRobotEncoders(telemetry);
//      stone detection
            fun.stoneDetectionBlue();
            double wallDis = robot.rightRangeSensor.getDistance(DistanceUnit.CM);
            fun.gyroStrafe(3.14, 0, 145 - wallDis, .85, 10);
//            while (robot.rightRangeSensor.getDistance(DistanceUnit.CM) < 65) {
//                fun.continuousGyroStrafe(1.57,0.0,.75);
//            }
            fun.autonomousParking(Functions.redOrBlue.BLUE, 3.14, 0.0);
            fun.gyroStrafe(3.14, 0.0, 100, .9, 10);
            robot.gripServo.setPosition(.7);
            robot.silverPlatter.setPower(.65);
            while (robot.extendedSwitch.getVoltage() < 3.2) {
                fun.waitMilis(5);
            }
            if (robot.extendedSwitch.getVoltage() > 3.0) {
                robot.silverPlatter.setPower(.0);
                fun.waitMilis(25);
            }
            robot.gripServo.setPosition(.5);
            fun.waitMilis(400);
            robot.lift.setTargetPosition(-200);
            robot.lift.setPower(-0.95);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fun.waitMilis(350);
            robot.silverPlatter.setPower(-.7);
            while (robot.retractedSwitch.getVoltage() < 3.2) {
                fun.waitMilis(5);
            }
            if (robot.retractedSwitch.getVoltage() > 3.0) {
                robot.silverPlatter.setPower(-0.05);
            }
            fun.gyroStrafe(4.71, 4.71, 5, 1, 5);
            fun.gyroStrafe(1.57, 4.71, 9, .7, 10);
//    foundation line up
            fun.foundationLinerUpper(.1);
            robot.gripServo.setPosition(1.0);
            robot.lift.setTargetPosition(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(.7);
            robot.rightFoundation.setPosition(.2);
            robot.leftFoundation.setPosition(.9);
            fun.waitMilis(1000);
            fun.gyroStrafe(4.71, 6.28, 35, .75, 10);
//            while (robot.rightRangeSensor.getDistance(DistanceUnit.CM) > 40) {
//                fun.continuousGyroStrafe(4.71,0,.8);
//            }
            fun.waitMilis(500);
            robot.leftFoundation.setPosition(0);
            robot.rightFoundation.setPosition(1.0);
            robot.lift.setTargetPosition(-350);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            fun.waitMilis(500);
            fun.gyroStrafe(0.0, 0.0, 20, 1, 5);
            while (robot.rightRangeSensor.getDistance(DistanceUnit.CM) < 50) {
                fun.continuousGyroStrafe(1.57, 0, 0.85);
            }
            robot.lift.setTargetPosition(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(.7);
            fun.autonomousParking(Functions.redOrBlue.BLUE,0,0);
            fun.gyroStrafe(0,0,40,1,10);
            fun.gyroStrafe(.785,.785,40,1,10);

//    red side autonomous
        } else if (robot.leftMarkerSwitch.getVoltage() < 3.0 && robot.rightMarkerSwitch.getVoltage() < 3.0) {
            telemetry.addData("Running Red", "Waiting For Start");
            telemetry.update();
            waitForStart();
//            fun.gyroStrafe(1.57, 1.57, 10, 0.5, 5);
//            fun.resetRobotEncoders(telemetry);
//            fun.stoneDetectionRed();
//        fun.autonomousParking(Functions.direction.REVERSE, Functions.redOrBlue.RED);
//            fun.gyroStrafe(3.1416,0,30,.3,10);
//            fun.gyroStrafe(1.571, 0, 20, .5, 10);
        } else {
            telemetry.addData("Left", robot.leftMarkerSwitch.getVoltage());
            telemetry.addData("Right", robot.rightMarkerSwitch.getVoltage());
            telemetry.update();
            fun.waitMilis(2000);
            throw new RuntimeException("Check Markers");
        }

    }


    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


}
