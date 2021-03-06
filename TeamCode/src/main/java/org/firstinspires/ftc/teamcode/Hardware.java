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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    /* Public OpMode members. */
    /**
     * Left Brain
     **/
    public DcMotor leftDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor leftIntake = null;
    public DcMotor silverPlatter = null;

    // Servos

    public Servo leftFoundation = null;
    public CRServo conveyorServo = null;

    // Sensors
    public RevColorSensorV3 bottomColorSensor = null;
    public RevColorSensorV3 leftColorSensor = null;
    public Rev2mDistanceSensor leftRangeSensor = null;
    public DistanceSensor leftFoundationSensor = null;
    public AnalogInput leftMarkerSwitch = null;
    public AnalogInput leftFoundationSwitch = null;
    public AnalogInput rightFoundationSwitch = null;


    /**
     * Right Brain
     **/

    // Motors
    public DcMotor rightDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor rightIntake = null;
    public DcMotor lift = null;
    // Servos
    public Servo rightFoundation = null;
    public Servo gripServo = null;

    // Sensors
    public RevColorSensorV3 rightColorSensor = null;
    public DistanceSensor rightFoundationSensor = null;
    public Rev2mDistanceSensor conveyorDistanceSensor = null;
    public Rev2mDistanceSensor rightRangeSensor = null;
    public AnalogInput rightMarkerSwitch = null;
    public AnalogInput retractedSwitch = null;
    public AnalogInput extendedSwitch = null;
    public AnalogInput liftSwitch = null;



    public static final double MID_SERVO = 0.5;
    public static final double MIN_SERVO = 0.0;
    public static final double MAX_SERVO = 1.0;
    public static final double ARM_UP_POWER = 0.25;
    public static final double ARM_DOWN_POWER = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware() {
    }

    double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    double WHEEL_DIAMETER_CENTIMETERS = 10.16;     // For figuring circumference
    double COUNTS_PER_CENTIMETER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTIMETERS * 3.1415);
    double DRIVE_SPEED = 1.0;
    double TURN_SPEED = 0.8;
    double DIST_PER_REV = (4 * 2.54 * Math.PI) / COUNTS_PER_MOTOR_REV;
    double openSwitch = 0.0;
    double closedSwitch = 3.25;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "fl");
        leftBackDrive = hwMap.get(DcMotor.class, "bl");
        rightDrive = hwMap.get(DcMotor.class, "fr");
        rightBackDrive = hwMap.get(DcMotor.class, "br");
        leftIntake = hwMap.get(DcMotor.class,"leftIntake");
        rightIntake = hwMap.get(DcMotor.class,"rightIntake");
        silverPlatter = hwMap.get(DcMotor.class,"silverPlatter");
        lift = hwMap.get(DcMotor.class,"liftMotor");


        // Set Drive Motor Direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize braking function
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        silverPlatter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        leftDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightDrive.setPower(0);
        rightBackDrive.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        leftFoundation = hwMap.get(Servo.class, "leftFoundation");
        rightFoundation = hwMap.get(Servo.class, "rightFoundation");
        conveyorServo = hwMap.get(CRServo.class, "conveyor");
        gripServo = hwMap.get(Servo.class, "gripperServo");

        leftFoundation.setPosition(0.95);
        rightFoundation.setPosition(0.0);

        // Define and initialize ALL installed Sensors.
        rightColorSensor = hwMap.get(RevColorSensorV3.class, "rightColorSensor");
        leftColorSensor = hwMap.get(RevColorSensorV3.class, "leftColorSensor");
        bottomColorSensor = hwMap.get(RevColorSensorV3.class, "bottomColorSensor");
        leftMarkerSwitch = hwMap.get(AnalogInput.class, "leftMarker");
        rightMarkerSwitch = hwMap.get(AnalogInput.class, "rightMarker");
        retractedSwitch = hwMap.get(AnalogInput.class, "retractSwitch");
        extendedSwitch = hwMap.get(AnalogInput.class, "extendSwitch");
        liftSwitch = hwMap.get(AnalogInput.class, "liftSwitch");
        conveyorDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "conveyorDistanceSensor");
        leftRangeSensor = hwMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        rightRangeSensor = hwMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");
        rightFoundationSwitch = hwMap.get(AnalogInput.class, "rightFoundationSwitch");
        leftFoundationSwitch = hwMap.get(AnalogInput.class, "leftFoundationSwitch");
    }

}

