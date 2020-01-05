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
    private ElapsedTime runtime = new ElapsedTime();
    double oldTime = 0.0;
    Functions fun = new Functions(robot, imu);

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
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Set up our telemetry dashboard
        Functions fun = new Functions(robot, imu);
        fun.resetEncoders();
        fun.waitMilis(50);
        composeTelemetry();
        telemetry.addLine();
        telemetry.addData("Robot", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.leftDrive.setPower(v1);
            robot.rightDrive.setPower(v2);
            robot.leftBackDrive.setPower(v3);
            robot.rightBackDrive.setPower(v4);

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
            boolean right_trigger1;
            boolean left_trigger1;
            if (gamepad1.right_trigger > 0.5) {
                right_trigger1 = true;
            } else {
                right_trigger1 = false;
            }
            if (gamepad1.left_trigger > 0.5) {
                left_trigger1 = true;
            } else {
                left_trigger1 = false;
            }

            if (right_trigger1) {
                fun.foundationGrabber(Functions.foundationPos.CLOSED);
            } else {
                fun.foundationGrabber(Functions.foundationPos.OPEN);
            }
            /** Gunner **/

            boolean automated = true;
           if (gamepad2.a) {
               automated = false;
           } else {
               automated = true;
           }
           if (automated)  {
               gamepad2.
           }
            double closed = 1.0;
            double open = 0.5;
        }
        /**
         * May need to change these values for both & ask Josh about controls
         */
        //buttons a & b

        double closed = 1.0;
        double open = 0.5;

    }
    private void resetGunnerEncoders() {
        telemetry.addData("Initialized", "Resetting Encoders");
        telemetry.update();
        fun.waitMilis(100);
        telemetry.addData("Resetting Silver Platter", "Resetting Encoders");
        telemetry.update();
        robot.lift.setPower(.2);
        fun.waitMilis(250);
        robot.lift.setPower(0);
        robot.silverPlatter.setPower(.2);
        fun.waitMilis(250);
        robot.silverPlatter.setPower(-0.2);
        fun.waitMilis(100);
        while (robot.retractedSwitch.getVoltage() < 3.3) {
            robot.silverPlatter.setPower(-0.1);
        }
        robot.silverPlatter.setPower(0.0);
        telemetry.addData("Silver Platter Reset", "Resetting Encoders");
        telemetry.update();
        robot.silverPlatter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fun.waitMilis(100);
        telemetry.addData("Resetting Lift", "Resetting Encoders");
        while (robot.liftSwitch.getVoltage() < 3.3) {
            robot.lift.setPower(-0.05);
        }
        robot.lift.setPower(0.0);
        telemetry.addData("Lift Reset", "Resetting Encoders");
        telemetry.update();
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fun.waitMilis(100);
        telemetry.addData("Resume Playing", "Have a nice day");
        telemetry.update();

    }



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



