package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

@Autonomous(name = "Autonomous", group = "linearOpMode")

public class Autonomous_OpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor fr;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
    }
}
