package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="EncodeTeleOp", group="ahert")  // @Autonomous(...) is the other common choice
//@Disabled

public class EncodeTeleOp extends OpMode {

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////

    double threshold = 0.1;
    int armCoefficient = 1;         // This integer switches to -1 when a button is pushed, and it
                                    // is used to reverse one of the arms for both adjustment and
                                    // as a failsafe in case one of the arms reverses unintentionally

    boolean steadyDown = false;     // Variable used for the launcher

    //int backShieldMode = 1;

    //boolean red = false;
    //boolean blue = false;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: andymark Motor Encoder
    static final double     DRIVE_SPEED             = 1.0;
    private ElapsedTime runtime = new ElapsedTime();


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //DECLARE MOTOR AND SERVO OBJECTS
    ////////////////////////////////////////////////////////////////////////////////////////////////

    //DcMotor motorRightFront;        // These four motors move the wheels.
    //DcMotor motorLeftFront;
    //DcMotor motorRightBack;
    //DcMotor motorLeftBack;

    //DcMotor kraken;                // This motors move the ball intake called the "Kraken".

    DcMotor launcher;                // The launcher functions to shot the balls into the goal

    //Servo backShield;               // This servo loads the particle (Balls) into the ball launcher

   // DcMotor LEDs;


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //INITIALIZE ROBOT
    ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init() {

		/*
         * This is where the motor and servo objects declared above are actually linked to the
         * motors and servos in the configuration file in the phone.
		 */

        //motorRightFront = hardwareMap.dcMotor.get("R1");
        //motorRightBack = hardwareMap.dcMotor.get("R2");
        //motorLeftFront = hardwareMap.dcMotor.get("L1");
        //motorLeftBack = hardwareMap.dcMotor.get("L2");
        //motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        //motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        //motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //kraken = hardwareMap.dcMotor.get("Kr");

        launcher = hardwareMap.dcMotor.get("L");

        //backShield = hardwareMap.servo.get("BS");       //Channel 1

        //  LEDs = hardwareMap.dcMotor.get("L");

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //MAIN LOOPING PROGRAM
    ////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void loop() {

		/*
         * CONTROLS OVERVIEW:
		 *
		 * GAMEPAD 1 controls the WHEELS and the "kraken" (ball intake system)
		 *
		 *      The robot is configured to be a "tank drive" such that the left analog stick
		 *      controls the wheels on the left, and the right stick controls the wheels on the
		 *      right.
		 *
		 *      There are also buttons to adjust the maximum speed of the wheels.
		 *
		 *      (right bumper)  = TURBO MODE for wheels (up to 100% power on the wheel motors)
		 *
		 *      The wheel motors max out at 50% power if neither button is pushed.
		 *
		 * GAMEPAD 2 controls the Ball loader and launcher
		 *
		 *      (d_pad up ) = Loads ball top launcher, down if not pressed
		 *      (right bumper) = Spins launcher motor when pressed
		  *     (right trigger) = reverses laucher motor for position correction
		 */

        //LEDs.setPower(0.75); //Gives 9V to the LEDs on the front of the robot

        //Ball Loader CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////////
        //if (gamepad2.left_bumper) {                     // Change shield mode to 0
            //backShieldMode = 1;
        //}else {backShieldMode = 0;                // Change shield mode to 1
        //}

       // backShield.setPosition(backShieldMode);     // Set servo to the shield mode (0 = up, 1 = down)

        //telemetry.addData("steadyDown Variable", steadyDown);


        //Launcher CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.addData("Path0",  "Starting at %7d :%7d",
                launcher.getCurrentPosition();
        telemetry.update();

        if (gamepad2.right_bumper)                   // Launcher control
            encoderDrive(DRIVE_SPEED,  1120, 2.0);
        //else if (gamepad2.right_trigger > threshold)
            //launcher.setPower(-1.0);
        else
            launcher.setPower(0);}

        //Kraken CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////

        //if (gamepad1.left_bumper)                   // Kraken control
            //kraken.setPower(1.0);
        //else if (gamepad1.left_trigger > threshold)
            //kraken.setPower(-1.0);
        //else
            //kraken.setPower(0);

            //WHEEL CONTROL
            ////////////////////////////////////////////////////////////////////////////////////////

            //double left = -gamepad1.left_stick_y;
            //double right = -gamepad1.right_stick_y;

            // clip the right/left values so that the values never exceed +/- 1

            //right = Range.clip(right, -1, 1);
            //left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            // right = scaleInput(right);
            // left = scaleInput(left);

            //if (gamepad1.right_bumper) {                        //  If the right bumper is pushed,
                //motorRightFront.setPower(right);                // enable turbo mode for the wheels
                //motorRightBack.setPower(right);                 // (up to 100% speed).
                //motorLeftFront.setPower(left);                  //
                //motorLeftBack.setPower(left);                   //

            //} else {                                            //  Otherwise, make the wheels go at
                //motorRightFront.setPower(0.5 * right);          // normal speed (up to 50% speed)
                //motorRightBack.setPower(0.5 * right);           //
                //motorLeftFront.setPower(0.5 * left);            //
               // motorLeftBack.setPower(0.5 * left);             //
           // }
        /////////////////////////// encoderDrive Method
    public void encoderDrive(double speed,
                             double leftInches,
                             double timeoutS) {
        int newLeftTarget;
        //int newRightTarget;

        // Ensure that the opmode is still active
        if (gamepad2.right_bumper) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = launcher.getCurrentPosition() ;//+ (int)(leftInches * COUNTS_PER_INCH);
            //newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            launcher.setTargetPosition(newLeftTarget);
            //robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            launcher.setPower(Math.abs(speed));
            //robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (gamepad2.right_bumper &&
                (runtime.seconds() < timeoutS) &&
                        (launcher.isBusy()));

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d",
                        launcher.getCurrentPosition();
                        //robot.rightMotor.getCurrentPosition());
                        //telemetry.update();
            }

            // Stop all motion;
             launcher.setPower(0);
            //robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

           //sleep(250);   // optional pause after each move
        }
        }

