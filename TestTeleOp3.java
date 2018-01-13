package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

public class TestTeleOp3 extends OpMode {

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////

    double threshold = 0.1;
    int armCoefficient = 1;         // This integer switches to -1 when a button is pushed, and it
                                    // is used to reverse one of the arms for both adjustment and
                                    // as a failsafe in case one of the arms reverses unintentionally

    boolean steadyDown = false;     // Variable used for the antlers

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


    DcMotor armLeft;                // These two motors move the arm.
    DcMotor armRight;

    DcMotor antlers;                // The antlers function as wheelie bars which help keep the robot
                                    // from flipping backwards while climbing.

    DcMotor LEDs;

    Servo leftZipLine;              // These servos help score zipline climbers, and they are
    Servo rightZipLine;             // positioned on either side of the robot.

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

        armLeft = hardwareMap.dcMotor.get("A1");
        armRight = hardwareMap.dcMotor.get("A2");
        armLeft.setDirection(DcMotor.Direction.REVERSE);

        antlers = hardwareMap.dcMotor.get("T");
        LEDs = hardwareMap.dcMotor.get("L");

        leftZipLine = hardwareMap.servo.get("LZ");      //Channel 1
        rightZipLine = hardwareMap.servo.get("RZ");     //Channel 2
        backShield = hardwareMap.servo.get("BS");       //Channel 6


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

        LEDs.setPower(0.75); //Gives 9V to the LEDs on the front of the robot

        //ARM CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////////
        double armPower = gamepad2.left_stick_y;

        armPower = Range.clip(armPower, -1, 1);

        armPower = scaleInput(armPower);

        if (gamepad2.y) {                                        // If the y button is pushed,
            armLeft.setPower(armPower);                          // the arm moves at up to 100% speed
            armRight.setPower(armCoefficient * armPower);
        } else {                                                 //Otherwise, the max is 50% speed
            armLeft.setPower(0.5 * armPower);
            armRight.setPower(armCoefficient * 0.5 * armPower);
        }

        if(gamepad2.right_bumper) {
            armCoefficient = -1;
        }else{
            armCoefficient = 1;
        }


        if(gamepad1.x) {                                   // If x is pressed, that disables the wheel control
            antlers.setPower(0.5*gamepad1.left_stick_y);   // and enables precision wheelie bar control for when
                                                           // you use the all clear signal
        }else{

            //WHEEL CONTROL
            ////////////////////////////////////////////////////////////////////////////////////////

            double left = -gamepad1.right_stick_y;
            double right = -gamepad1.left_stick_y;

            // clip the right/left values so that the values never exceed +/- 1

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            right = scaleInput(right);
            left = scaleInput(left);

            if (gamepad1.right_bumper) {                        //  If the right bumper is pushed,
                motorRightFront.setPower(right);                // enable turbo mode for the wheels
                motorRightBack.setPower(right);                 // (up to 100% speed).
                motorLeftFront.setPower(left);                  //
                motorLeftBack.setPower(left);                   //
            } else if (gamepad1.right_trigger >= threshold) {   //  If the right trigger is pushed,
                motorRightFront.setPower(0.25 * right);         // enable slow mode for the wheels
                motorRightBack.setPower(0.25 * right);          // (up to 25% speed).
                motorLeftFront.setPower(0.25 * left);           //
                motorLeftBack.setPower(0.25 * left);            //
            } else {                                            //  Otherwise, make the wheels go at
                motorRightFront.setPower(0.5 * right);          // normal speed (up to 50% speed)
                motorRightBack.setPower(0.5 * right);           //
                motorLeftFront.setPower(0.5 * left);            //
                motorLeftBack.setPower(0.5 * left);             //
            }                                                   //

            //ANTLER/WHEELIE BAR CONTROL
            ////////////////////////////////////////////////////////////////////////////////////////

            if (gamepad1.b)
                steadyDown = true;
            if (gamepad1.left_trigger > threshold || gamepad1.left_bumper)
                steadyDown = false;
            if (steadyDown) {                               // Wheelie bar toggle so that it can
                antlers.setPower(-0.2);                         // drive by itself while climbing
            } else {
                if (gamepad1.left_bumper)                   // Standard wheelie bar control
                    antlers.setPower(0.5);
                else if (gamepad1.left_trigger > threshold)
                    antlers.setPower(-0.5);
                else
                    antlers.setPower(0);
            }
        }


        //ZIPLINE SERVO CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////////
        if (gamepad1.dpad_right && ziplineMode < 1) { //Change between zipline modes
            ziplineMode++;
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (gamepad1.dpad_left && ziplineMode > -1) {
            ziplineMode--;
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
                                                    // Zipline Modes:
                                                    //
        if (ziplineMode == -1) {                    // Mode -1:
            leftZiplineState = 0;                   // Left zipline servo out.
            rightZiplineState = 0.5;                // Right zipline servo in.
                                                    //
        } else if (ziplineMode == 0) {              // Mode 0:
            leftZiplineState = 0.5;                 // Left zipline servo in.
            rightZiplineState = 0.5;                // Right zipline servo in.
                                                    //
        } else {                                    // Mode 1:
            leftZiplineState = 0.5;                 // Left zipline servo in.
            rightZiplineState = 1;                  // Right zipline servo out.
        }

        leftZipLine.setPosition(leftZiplineState);  // Set servos to the correct positions
        rightZipLine.setPosition(rightZiplineState);


        //BACK SHIELD CONTROL
        ////////////////////////////////////////////////////////////////////////////////////////////
        if (gamepad1.dpad_up) {                     // Change shield mode to 0
            backShieldMode = 0;
        }
        if (gamepad1.dpad_down) {                   // Change shield mode to 1
            backShieldMode = 1;
        }

        backShield.setPosition(backShieldMode);     // Set servo to the shield mode (0 = up, 1 = down)

        telemetry.addData("steadyDown Variable", steadyDown);
    }

    @Override
    public void stop() {

    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}


