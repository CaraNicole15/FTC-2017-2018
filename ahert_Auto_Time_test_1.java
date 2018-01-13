/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_Time_test_1", group="ahert" )
//@Disabled
public class ahert_Auto_Time_test_1 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //DECLARE MOTOR AND SERVO OBJECTS
    ////////////////////////////////////////////////////////////////////////////////////////////////

    DcMotor motorRightFront;        // These four motors move the wheels.
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;

    DcMotor kraken;                // This motors move the ball intake called the "Kraken".

    DcMotor launcher;                // The launcher functions to shot the balls into the goal

    Servo backShield;               // This servo loads the particle (Balls) into the ball launcher

    // DcMotor LEDs;


    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode()throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // robot.init(hardwareMap);

        /*
         * This is where the motor and servo objects declared above are actually linked to the
         * motors and servos in the configuration file in the phone.
		 */
        //initialize motors
        motorRightFront = hardwareMap.dcMotor.get("R1");
        motorRightBack = hardwareMap.dcMotor.get("R2");
        motorLeftFront = hardwareMap.dcMotor.get("L1");
        motorLeftBack = hardwareMap.dcMotor.get("L2");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        kraken = hardwareMap.dcMotor.get("Kr");
        launcher = hardwareMap.dcMotor.get("L");


        //initialize servo
        backShield = hardwareMap.servo.get("BS");       //Channel 1
        backShield.setPosition(0.0);

        //  LEDs = hardwareMap.dcMotor.get("L");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive backward for 3 seconds

        motorLeftFront.setPower(-FORWARD_SPEED);
        motorRightFront.setPower(-FORWARD_SPEED);
        motorLeftBack.setPower(-FORWARD_SPEED);
        motorRightBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        sleep(500);

        //Step 2: launch ball

        launcher.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        launcher.setPower(0.0);

        //Step 3:  Load Ball launcher

        backShield.setPosition(1.0);
        sleep(1000);
        backShield.setPosition(0.0);
        sleep(1000);

        // Step 4:  Launch ball #2

        launcher.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        launcher.setPower(0.0);

        // Step 4:  kraken run
        kraken.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        kraken.setPower(0.0);

        //Step 3:  Load Ball #3 to launcher

        backShield.setPosition(1.0);
        sleep(1000);
        backShield.setPosition(0.0);
        sleep(1000);

        // Step 5:  Launch ball #3
        launcher.setPower(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        launcher.setPower(0.0);

        //Step 6:  Spin right for 1.3 seconds
        //motorLeftFront.setPower(-TURN_SPEED);
        //motorRightFront.setPower(TURN_SPEED);
        //motorLeftBack.setPower(-TURN_SPEED);
        //motorRightBack.setPower(TURN_SPEED);

        //runtime.reset();
        //while (opModeIsActive() && (runtime.seconds() < 0.75)) {
          //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
          //  telemetry.update();
        //}

        //Step :  Drive Backwards for 1 Second
        motorLeftFront.setPower(-FORWARD_SPEED);
        motorRightFront.setPower(-FORWARD_SPEED);
        motorLeftBack.setPower(-FORWARD_SPEED);
        motorRightBack.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

            // Step 4:  Stop and close the claw.
            //robot.leftMotor.setPower(0);
            // robot.rightMotor.setPower(0);
            // robot.leftClaw.setPosition(1.0);
            //robot.rightClaw.setPosition(0.0);

            //telemetry.addData("Path", "Complete");
            //telemetry.update();
            //sleep(1000);}

    }


}