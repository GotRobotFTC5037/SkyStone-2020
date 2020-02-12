package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name = "TeleOopSKSKS", group = "Linear Opmode")
public class TeleOopSKSKS extends LinearOpMode {
    Hardware robot = new Hardware();
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private ElapsedTime runtime = new ElapsedTime();
    double oldTime = 0.0;
    double gunnerTime = 0.0;
    Functions fun = new Functions(robot, imu);
    double num;
    double num1;
    private boolean automated = true;
    private boolean reversed = false;
    int state;
    int autoState = 0;
    double driveSpeed = 1;
    int driveState = 0;
    int liftState = 0;
    int gunnerState;
    boolean liftMode = true;

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
        fun.resetDriveEncoders();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fun.waitMilis(50);
        composeTelemetry();
        telemetry.addLine();
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        fun.resetRobotEncoders(telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            if (robot.liftSwitch.getVoltage() > 3.0) {
                robot.lift.setMode(DcMotor.RunMode.RESET_ENCODERS);
            }
            telemetry.addData("Automated", autoState);
            telemetry.addData("Reversed", reversed);
            telemetry.update();
            fun.waitMilis(0);
            telemetry.addData("Speed", driveSpeed);
            telemetry.addData("LiftState", liftState);
            telemetry.update();
            if (gamepad2.a) {
                robot.gripServo.setPosition(0.5);
            } else if (gamepad2.b) {
                robot.gripServo.setPosition(1.0);
            }
            switch (autoState) {
                case 0: //Do not run
                    break;
                case 1:
                    if (robot.conveyorDistanceSensor.getDistance(DistanceUnit.CM) < 10.0) {
                        oldTime = runtime.milliseconds();
                        autoState++;
                    }
                    break;
                case 2:
                    if (runtime.milliseconds() > oldTime + 500) {
                        autoState++;
                    }
                    break;
                case 3: // Check for block + Button
                    if (robot.conveyorDistanceSensor.getDistance(DistanceUnit.CM) < 10.0 && gamepad2.y) {
                        autoState++;
                    }
                    break;
                case 4: // Open Gripper
                    robot.gripServo.setPosition(0.7);
                    // lastStateTime = getRuntime();
                    autoState++;
                    break;
                case 5: // Mover Silver Platter out until limit switch

                    robot.silverPlatter.setPower(.65);
                    // robot.silverPlatter.setPower(.65 * Math.pow(.4,getRuntime() - lastStateTime));
                    if (robot.extendedSwitch.getVoltage() > 3.2) {
                        robot.silverPlatter.setPower(.0);
                        fun.waitMilis(25);
                        autoState++;
                    }
                    break;
                case 6: // Close Gripper
                    robot.gripServo.setPosition(0.4);
                    oldTime = runtime.milliseconds();
                    autoState++;
                    break;
                case 7:
                    if (runtime.milliseconds() > oldTime + 300) {
                        autoState++;
                    }
                    break;
                case 8: // Lift to position
                    robot.lift.setTargetPosition(-175);
                    //robot.lift.setTargetPosition(-1000);
                    robot.lift.setPower(-0.95);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    autoState++;
                    break;
                case 9: // Wait For Lift
                    if (robot.lift.getCurrentPosition() < -100) {
                        autoState++;
                    } else {
                        robot.lift.setPower(-0.95);
                    }
                    break;
                case 10: // Retract Platter until retract switch
                    // robot.silverPlatter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.silverPlatter.setPower(-.7);
                    if (robot.retractedSwitch.getVoltage() > 3.2) {
                        robot.silverPlatter.setPower(-0.05);
                        autoState++;
                    }
                    break;
                default:
                    autoState = 0;
                    break;
            }

            if (gamepad1.start) {
                reversed = false;
            }
            if (gamepad1.back) {
                reversed = true;
            }
            double closed = 1.0;
            double open = 0.5;
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            if (gamepad1.dpad_up) {
                driveState--;
                fun.waitMilis(75);
            } else if (gamepad1.dpad_down) {
                driveState++;
                fun.waitMilis(75);
            } else if (gamepad1.dpad_right) {
                driveState = 0;
            } else if (gamepad1.dpad_left) {
                driveState = 5;
            }

            if (driveState == 0) {
                driveSpeed = 1;
            } else if (driveState == 1) {
                driveSpeed = .9;
            } else if (driveState == 2) {
                driveSpeed = .8;
            } else if (driveState == 3) {
                driveSpeed = .7;
            } else if (driveState == 4) {
                driveSpeed = .6;
            } else if (driveState == 5) {
                driveSpeed = .5;
            }

            if (reversed = true) {
                final double v1 = r * Math.cos(robotAngle) - rightX;
                final double v2 = r * Math.sin(robotAngle) + rightX;
                final double v3 = r * Math.sin(robotAngle) - rightX;
                final double v4 = r * Math.cos(robotAngle) + rightX;
                robot.leftDrive.setPower(-v1 * driveSpeed);
                robot.rightDrive.setPower(-v2 * driveSpeed);
                robot.leftBackDrive.setPower(-v3 * driveSpeed);
                robot.rightBackDrive.setPower(-v4 * driveSpeed);
            } else {
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;
                robot.leftDrive.setPower(v1 * driveSpeed);
                robot.rightDrive.setPower(v2 * driveSpeed);
                robot.leftBackDrive.setPower(v3 * driveSpeed);
                robot.rightBackDrive.setPower(v4 * driveSpeed);
            }

            if (gamepad1.b) {
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.a) {
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            boolean right_trigger1;
            boolean left_trigger1;
            boolean right_trigger2;
            boolean left_trigger2;
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
            if (gamepad2.right_trigger > 0.5) {
                right_trigger2 = true;
            } else {
                right_trigger2 = false;
            }
            if (gamepad2.left_trigger > 0.5) {
                left_trigger2 = true;
            } else {
                left_trigger2 = false;
            }

            if (right_trigger1) {
                robot.leftFoundation.setPosition(0.9);
                robot.rightFoundation.setPosition(0.2);
            } else {
                robot.leftFoundation.setPosition(0.0);
                robot.rightFoundation.setPosition(1.0);
            }


            /** Gunner **/
            if (gamepad2.start) {
                liftMode = true;
            } else if (gamepad2.back) {
                liftMode = false;
            }
            switch (gunnerState) {
                case 0:
                    if (gamepad2.dpad_up) {
                        liftState++;
                        gunnerTime = runtime.milliseconds();
                        gunnerState++;
                    } else if (gamepad2.dpad_down) {
                        liftState--;
                        gunnerTime = runtime.milliseconds();
                        gunnerState++;
                    } //125 Ticks per inch for lift
                    break;
                case 1:
                    if (runtime.milliseconds() > gunnerTime + 400) {
                        gunnerState++;
                    }
                    break;
                default:
                    gunnerState = 0;
                    break;
            }
            if (gamepad2.dpad_left) {
                autoState = 1;

            } else if (gamepad2.dpad_right) {
                autoState = 0;
            }

            if (gamepad2.a) {
                robot.gripServo.setPosition(0.5);
            } else if (gamepad2.b) {
                robot.gripServo.setPosition(1.0);
            }

            if (gamepad2.x) {
                liftState = 0;
            }
            if (autoState == 0) {
                if (right_trigger2) {
                    fun.intake(Functions.intake.IN, .5);
                    robot.conveyorServo.setPower(1.0);
                } else {
                    robot.leftIntake.setPower(.0);
                    robot.rightIntake.setPower(.0);
                    robot.conveyorServo.setPower(.0);
                    if (left_trigger2) {
                        fun.intake(Functions.intake.OUT, .5);
                        robot.conveyorServo.setPower(-1.0);
                    } else {
                        robot.leftIntake.setPower(.0);
                        robot.rightIntake.setPower(.0);
                        robot.conveyorServo.setPower(.0);
                    }
                }
                int liftValue = -433;
                int liftOffset = -190;
                if (liftMode = true) {
                    if (liftState == 0) {
                        robot.lift.setTargetPosition(0);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 1) {
                        robot.lift.setTargetPosition(liftValue * (liftState) + liftOffset);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 2) {
                        robot.lift.setTargetPosition(liftValue * (liftState) + liftOffset);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 3) {
                        robot.lift.setTargetPosition(liftValue * (1100) + liftOffset);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 4) {
                        robot.lift.setTargetPosition(liftValue * (liftState) + liftOffset);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 5) {
                        robot.lift.setTargetPosition(liftValue * (liftState) + liftOffset);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                } else {
                    if (liftState == 0) {
                        robot.lift.setTargetPosition(0);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 1) {
                        robot.lift.setTargetPosition(-800);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 2) {
                        robot.lift.setTargetPosition(-1233);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 3) {
                        robot.lift.setTargetPosition(-1690);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else if (liftState == 4) {
                        robot.lift.setTargetPosition(-2150);
                        robot.lift.setPower(.9);
                        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                }
                robot.silverPlatter.setPower(-gamepad2.left_stick_y * .5);
            } else if (autoState == 10) {

            } else if (autoState == 1 || autoState == 2) {
                robot.silverPlatter.setPower(-gamepad2.left_stick_y * .5);
                fun.intake(Functions.intake.IN, .75);
                robot.conveyorServo.setPower(1.0);
            } else if (autoState == 4 || autoState == 5) {
                robot.conveyorServo.setPower(0.2);
                fun.intake(Functions.intake.IN, .0);
            } else {
                robot.conveyorServo.setPower(0.0);
                fun.intake(Functions.intake.IN, .0);
            }
        }

    }

    /**
     * May need to change these values for both & ask Josh about controls
     */
    //buttons a & b
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
        while (robot.retractedSwitch.getVoltage() < robot.closedSwitch) {
            robot.silverPlatter.setPower(-0.1);
        }
        robot.silverPlatter.setPower(0.0);
        telemetry.addData("Silver Platter Reset", "Resetting Encoders");
        telemetry.update();
        robot.silverPlatter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fun.waitMilis(100);
        telemetry.addData("Resetting Lift", "Resetting Encoders");
        while (robot.liftSwitch.getVoltage() < robot.closedSwitch) {
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



