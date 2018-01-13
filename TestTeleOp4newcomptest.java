package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="TestTeleOp4newcomptest", group="ahert")  // @Autonomous(...) is the other common choice
//@Disabled

public class TestTeleOp4newcomptest extends OpMode {

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////

    double threshold = 0.1;
    int armCoefficient = 1;         // This integer switches to -1 when a button is pushed, and it
                                    // is used to reverse one of the arms for both adjustment and
                                    // as a failsafe in case one of the arms reverses unintentionally

    boolean steadyDown = false;     // Variable used for the launcher

    int backShieldMode = 1;

    //boolean red = false;
    //boolean blue = false;


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


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //INITIALIZE ROBOT
    ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init() {

		/*
         * This is where the motor and servo objects declared above are actually linked to the
         * motors and servos in the configuration file in the phone.
		 */

        motorRightFront = hardwareMap.dcMotor.get("R1");
        motorRightBack = hardwareMap.dcMotor.get("R2");
        motorLeftFront = hardwareMap.dcMotor.get("L1");
        motorLeftBack = hardwareMap.dcMotor.get("L2");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);


        kraken = hardwareMap.dcMotor.get("Kr");

        launcher = hardwareMap.dcMotor.get("L");

        backShield = hardwareMap.servo.get("BS");       //Channel 1

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
        if (gamepad2.left_bumper) {                     // Change shield mode to 0
            backShieldMode = 1;
        }else {backShieldMode = 0;                // Change shield mode to 1
        }

        backShield.setPosition(backShieldMode);     // Set servo to the shield mode (0 = up, 1 = down)

        telemetry.addData("steadyDown Variable", steadyDown);


        //Launcher CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////

        if (gamepad2.right_bumper)                   // Launcher control
            launcher.setPower(1.0);
        else if (gamepad2.right_trigger > threshold)
            launcher.setPower(-1.0);
        else
            launcher.setPower(0);

        //Kraken CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////

        if (gamepad1.left_bumper)                   // Kraken control
            kraken.setPower(1.0);
        else if (gamepad1.left_trigger > threshold)
            kraken.setPower(-1.0);
        else
            kraken.setPower(0);

            //WHEEL CONTROL
            ////////////////////////////////////////////////////////////////////////////////////////

            double left = -gamepad1.left_stick_y;
            double right = -gamepad1.right_stick_y;

            // clip the right/left values so that the values never exceed +/- 1

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            // right = scaleInput(right);
            // left = scaleInput(left);

            if (gamepad1.right_bumper) {                        //  If the right bumper is pushed,
                motorRightFront.setPower(right);                // enable turbo mode for the wheels
                motorRightBack.setPower(right);                 // (up to 100% speed).
                motorLeftFront.setPower(left);                  //
                motorLeftBack.setPower(left);                   //

            } else {                                            //  Otherwise, make the wheels go at
                motorRightFront.setPower(0.5 * right);          // normal speed (up to 50% speed)
                motorRightBack.setPower(0.5 * right);           //
                motorLeftFront.setPower(0.5 * left);            //
                motorLeftBack.setPower(0.5 * left);             //
            }                                                   //
        }
    }