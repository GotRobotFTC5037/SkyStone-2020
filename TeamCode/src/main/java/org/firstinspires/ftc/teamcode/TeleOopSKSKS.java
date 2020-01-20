package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name = "TeleOopSKSKS", group = "Linear Opmode")
public class TeleOopSKSKS extends LinearOpMode {
    HardwareTest robot = new HardwareTest();
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double oldTime = 0.0;
    double num = 0;
    double num1 = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Functions fun = new Functions(robot, imu);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Set up our telemetry dashboard
        composeTelemetry();

        telemetry.addData("Hydroflask", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {

//            double blPower;
//            double brPower;
//            double flPower;
//            double frPower;
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI) / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.leftDrive.setPower(v1);
            robot.rightDrive.setPower(v2);
            robot.leftBackDrive.setPower(v3);
            robot.rightBackDrive.setPower(v4);
            //dpad up/down


            if (time > oldTime + 0.01) {
                oldTime = time;

                double currentPos;
                double resetPos = 1.0;
                currentPos = robot.armServo.getPosition();

                if (gamepad2.dpad_up) {
                    //robot.armServo.setPosition(currentPos - 0.005)
                    currentPos -= 0.0075;
                } else if (gamepad2.dpad_down) {
                    //robot.armServo.setPosition(currentPos + 0.005);
                    currentPos += 0.004;
                } else if (gamepad2.dpad_right) {
                    //robot.armServo.setPosition(resetPos);
                    currentPos = resetPos;
                } else {
                    robot.armServo.setPosition(currentPos);
                }
                currentPos = Math.min(currentPos, 1.0);
                currentPos = Math.max(currentPos, 0.165);
                robot.armServo.setPosition(currentPos);
//Josh is cool
// Raymond is less cool :)
// still cool tho
                telemetry.addData("ServoPos", robot.armServo.getPosition());
                telemetry.addData("ServoPosGrip", robot.gripperServo.getPosition());
                telemetry.update();
            }

            //buttons a & b

            double open = 1.0;
            double closed = 0.2;
            boolean right_trigger1;
            boolean left_trigger1;
            if (gamepad1.right_trigger > 0.5) {
                right_trigger1 = true;
            } else {
                right_trigger1 = false;
            }
            if (gamepad1.left_trigger > 0.5) {
                left_trigger1 = true;

            }

            if (right_trigger1) {
              //  fun.foundationGrabber(Functions.foundationPos.CLOSED);
            } else {
                //fun.foundationGrabber(Functions.foundationPos.OPEN);
            }

            if (gamepad2.y) {
                num++;
                if (num % 2 == 0) {
                    telemetry.addLine("Even");
                    fun.foundationGrabber(Functions.foundationPos.CLOSED);
                    fun.waitMilis(500);
                } else if (!(num % 2 == 0)) {
                    telemetry.addLine("Odd");
                    fun.foundationGrabber(Functions.foundationPos.OPEN);
                    fun.waitMilis(500);
                }
                telemetry.update();
                fun.waitMilis(100);
            }

            if (gamepad1.a) {
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad1.b) {
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.a) {
                robot.gripperServo.setPosition(closed);
            } else if (gamepad2.b) {
                robot.gripperServo.setPosition(open);
            }
//            if (right_trigger = true) {
//                robot.rightFoundation.setPosition(closed);
//                robot.leftFoundation.setPosition(closed);
//            } else if (left_trigger = true) {
//                robot.rightFoundation.setPosition(open);
//                robot.leftFoundation.setPosition(open);
//            }
        }


//            double drive_right = gamepad1.right_stick_y;
//            double drive_left = gamepad1.left_stick_y;

//            bl.setPower(drive_left);
//            br.setPower(drive_right);
//            fl.setPower(drive_left);
//            fr.setPower(drive_right);

    }
    // run until the end of the match (driver presses STOP)


//    public void gripperPos () {
//        double closed = 0.0;
//        double open = 0.5;
//        while (opModeIsActive()) {
//            if (gamepad2.a) {
//                robot.armServo.setPosition(open);
//                return;
//            }
//            else if (gamepad2.b) {
//                robot.armServo.setPosition(closed);
//                return;
//            }
//            else {
//                robot.armServo.setPosition(closed);
//                return;
//            }
//        }
//    }
//    public void armPos () {
//double currentPos;
//double resetPos = 1.0;
//while (opModeIsActive()) {
//    currentPos = robot.armServo.getPosition();
//    if (gamepad2.dpad_up) {
//        robot.armServo.setPosition(currentPos - 0.2);
//        return;
//    }
//    else if (gamepad2.dpad_down) {
//        robot.armServo.setPosition(currentPos + 0.2);
//        return;
//    }
//    else if (gamepad2.dpad_right) {
//        robot.armServo.setPosition(resetPos);
//    }
//    else {
//        robot.armServo.setPosition(currentPos);
//        return;
//    }
//        }
//    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}



