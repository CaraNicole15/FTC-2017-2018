package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by bwebb on 12/3/2016.
 */
@TeleOp(name="TestTeleOp4P1", group="ahert")  // @Autonomous(...) is the other common choice
//@Disabled

public class TestTeleOp4P1 extends OpMode{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////

    double threshold = 0.1;
    int armCoefficient = 1;         // This integer switches to -1 when a button is pushed, and it
    // is used to reverse one of the arms for both adjustment and
    // as a failsafe in case one of the arms reverses unintentionally

    boolean steadyDown = false;     // Variable used for the launcher

    int backShieldMode = 1;

    boolean red = false;
    boolean blue = false;


    int ziplineMode = 0;            // Variables used for controlling the zipline servos
    double leftZiplineState = 0;
    double rightZiplineState = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //DECLARE MOTOR AND SERVO OBJECTS
    ////////////////////////////////////////////////////////////////////////////////////////////////

    DcMotor motorRightFront;        // These four motors move the wheels.
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;

    //Motor Kraken;                // This motors move the ball intake called the "Kraken".

    DcMotor kraken;                // The kraken function as wheelie bars which help keep the robot
    // from flipping backwards while climbing.//

    DcMotor launcher;                // The launcher function as wheelie bars which help keep the robot
    // from flipping backwards while climbing.//

    // DcMotor LEDs;

    //Servo LDR;              // This servo loads the particle (Balls) into the ball launcher

    // Servo rightZipLine;

    Servo backShield;               // This servo moves the adjustable back shield.

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


        //XLR8R = hardwareMap.dcMotor.get("XL");

        //armRight = hardwareMap.dcMotor.get("A2");
        // armLeft.setDirection(DcMotor.Direction.REVERSE);

        kraken = hardwareMap.dcMotor.get("Kr");

        launcher = hardwareMap.dcMotor.get("L");

        //  LEDs = hardwareMap.dcMotor.get("L");

        //  LDR = hardwareMap.servo.get("LDR");      //Channel 1  //used to be leftZipline

        // rightZipLine = hardwareMap.servo.get("RZ");     //Channel 2

        backShield = hardwareMap.servo.get("BS");       //Channel 1


    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //MAIN LOOPING PROGRAM
    ////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void loop() {

		/*
         * CONTROLS OVERVIEW:
		 *
		 * GAMEPAD 1 controls the WHEELS and the CLIMBING MECHANISM.
		 *
		 *      The robot is configured to be a "tank drive" such that the left analog stick
		 *      controls the wheels on the left, and the right stick controls the wheels on the
		 *      right.
		 *
		 *      There are also buttons to adjust the maximum speed of the wheels.
		 *
		 *      (right bumper)  = TURBO MODE for wheels (up to 100% power on the wheel motors)
		 *
		 *      (right trigger) = SLOW MODE for wheels (up to 25% power on the wheel motors)
		 *
		 *      The wheel motors max out at 50% power if neither button is pushed.
		 *
		 *      (dpad left)     = Extend left zipline climber servo if neither are extended,
		 *                        or retract right zipline climber servo if it is extended.
		 *
		 *      (dpad right)    = Extend right zipline climber servo if neither are extended,
		 *                        or retract left zipline climber servo if it is extended.
		 *
		 *      (dpad up/down)  = Raise/lower back shield
		 *
		 *      (left bumper)   = Raise wheelie bar
		 *
		 *      (left trigger)  = Lower wheelie bar while pressed
		 *
		 *      (b)             = Toggle to slowly lower wheelie bar (used during climbing to apply
		 *                        a force to prevent the robot from flipping backwards
		 *
		 *      (x) + (left stick y) = Precision control of wheelie bar
		 *
		 * GAMEPAD 2 controls the ARM, HAND, and KRAKAN
		 *
		 *      (left stick y)  = arm control (arm motors max out at 50% power by default)
		 *
		 *      (Y button)      = TURBO/HANGING MODE for the arm (arm motors max out at 100% power)
		 *
		 *      (right bumper) = reverse arm motor as a failsafe
		 */

        //LEDs.setPower(0.75); //Gives 9V to the LEDs on the front of the robot

        //Ball Loader CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////////
        if (gamepad1.dpad_up) {                     // Change shield mode to 0
            backShieldMode = 1;
        }else {backShieldMode = 0;                // Change shield mode to 1
        }

        backShield.setPosition(backShieldMode);     // Set servo to the shield mode (0 = up, 1 = down)

        telemetry.addData("steadyDown Variable", steadyDown);

        //Kraken CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////

        if (gamepad1.left_bumper)                   // Kraken control
            kraken.setPower(1.0);
        else if (gamepad1.left_trigger > threshold)
            kraken.setPower(-1.0);
        else
            kraken.setPower(0);

        //Launcher CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////

        if (gamepad1.right_bumper)                   // Launcher control
            launcher.setPower(1.0);
        else if (gamepad1.right_trigger > threshold)
            launcher.setPower(-1.0);
        else
            launcher.setPower(0);

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
       /// } else if (gamepad1.right_trigger >= threshold) {   //  If the right trigger is pushed,
           // motorRightFront.setPower(0.25 * right);         // enable slow mode for the wheels
          //  motorRightBack.setPower(0.25 * right);          // (up to 25% speed).
          //  motorLeftFront.setPower(0.25 * left);           //
           // motorLeftBack.setPower(0.25 * left);            //
        } else {                                            //  Otherwise, make the wheels go at
            motorRightFront.setPower(0.5 * right);          // normal speed (up to 50% speed)
            motorRightBack.setPower(0.5 * right);           //
            motorLeftFront.setPower(0.5 * left);            //
            motorLeftBack.setPower(0.5 * left);             //
        }                                                   //
    }
}

