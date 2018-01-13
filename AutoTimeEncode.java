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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoTimeEncode", group="ahert" )
//@Disabled
public class AutoTimeEncode extends LinearOpMode {


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



    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV ) /
                                                       (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double Drive_SPEED = 0.6;
    static final double Launcher_SPEED = 1.0;


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

        //reset encoders on all motors
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorRightFront.getCurrentPosition(),
                motorRightBack.getCurrentPosition(),
                motorLeftFront.getCurrentPosition(),
                motorLeftBack.getCurrentPosition(),
                launcher.getCurrentPosition());
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        encoderDrive(-Drive_SPEED,38,38,4);//drive to align for shooting balls
        encoderLauncher(Launcher_SPEED,1,0.5); //launch ball #1
        LoadBall(1); //load ball #2
        encoderLauncher(Launcher_SPEED,1,0.5); //launch ball #2
        encoderDrive(-Drive_SPEED,20,20,2);}//park on platform under vortex goal



    public void encoderLauncher(double speed,
                             double rotations,
                             double runtimeS) {
        int newLauncherTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLauncherTarget = launcher.getCurrentPosition() + (int)(rotations * COUNTS_PER_MOTOR_REV);

            launcher.setTargetPosition(newLauncherTarget);

            // Turn On RUN_TO_POSITION
            launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            launcher.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and motor is running.
            while (opModeIsActive() &&
                    (runtime.seconds() < runtimeS) &&
                    (launcher.isBusy()));
            }

            // Stop all motion;
            launcher.setPower(0);

            sleep(100);   // pause after each move
        }

    public void LoadBall(double power)
    {
        backShield.setPosition(1.0);
        sleep(1000);
        backShield.setPosition(0.0);
        sleep(1000);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = motorLeftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = motorLeftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = motorRightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = motorRightBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorLeftBack.setTargetPosition(newLeftBackTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);
            motorRightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeftFront.setPower(Math.abs(speed));
            motorLeftBack.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));
            motorRightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and all motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (       motorLeftFront.isBusy() &&
                            motorLeftBack.isBusy() &&
                            motorRightFront.isBusy() &&
                            motorRightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget, newLeftBackTarget,
                        newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeftFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition(),
                        motorRightFront.getCurrentPosition(),
                        motorRightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            sleep(500);   // pause after each move
        }
    }

}


