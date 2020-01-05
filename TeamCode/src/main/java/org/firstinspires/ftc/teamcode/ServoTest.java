package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;


@TeleOp(name = "Frankie", group = "Test")

public class ServoTest extends LinearOpMode {
    //private Servo conveyorServo;
    //private Servo gripServo;
    //private DcMotor silverTrayMotor;
    //private DcMotor intakeLeft;
    //private DcMotor intakeRight;
    private AnalogInput testSwitch = null;
    private AnalogInput testSwitch2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //conveyorServo = hardwareMap.get(Servo.class, "servo");
        //gripServo = hardwareMap.get(Servo.class, "gripServo");
        //silverTrayMotor = hardwareMap.get(DcMotor.class, "silverTrayMotor");
        //intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
       // intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        testSwitch = hardwareMap.get(AnalogInput.class, "switch");
        testSwitch2 = hardwareMap.get(AnalogInput.class,"switch2");


        waitForStart();

        while (opModeIsActive()) {



//            if (gamepad1.a) {
//                gripServo.setPosition(1.0);
//            } else if (gamepad1.b) {
//                gripServo.setPosition(0.5);
//            }
//            conveyorServo.setPosition((gamepad1.left_stick_y + 1) / 2);
//            silverTrayMotor.setPower(gamepad1.right_stick_y);
//
//            if (gamepad1.right_trigger > 0.5) {
//                intakeLeft.setPower(0.5);
//                intakeRight.setPower(-0.5);
//            } else {
//                intakeLeft.setPower(0.0);
//                intakeRight.setPower(0.0);
//            }
//
//            telemetry.addData("Servo Data", conveyorServo.getPosition());
//            telemetry.addData("Motor Data", silverTrayMotor.getPower());
//            telemetry.update();
        }
    }
}
