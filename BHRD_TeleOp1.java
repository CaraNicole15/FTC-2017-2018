package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Carina on 10/29/2017.
 */

@TeleOp(name="BHRD_TeleOp1", group="BHRD")  // @Autonomous(...) is the other common choice
//@Disabled

public class BHRD_TeleOp1 extends OpMode {


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //DECLARE MOTOR AND SERVO OBJECTS
    ////////////////////////////////////////////////////////////////////////////////////////////////

    DcMotor RightFront;        // These four motors move the wheels.
    DcMotor RightBack;
    DcMotor LeftFront;
    DcMotor LeftBack;

    DcMotor Lift;

    Servo GlyphArm1;
    Servo GlyphArm2;

    Servo JewelArm;
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //INITIALIZE ROBOT
   ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init() {

		/*
         * This is where the motor and servo objects declared above are actually linked to the
         * motors and servos in the configuration file in the phone.
		 */

        RightFront = hardwareMap.dcMotor.get("RF");
        RightBack = hardwareMap.dcMotor.get("RB");
        LeftFront = hardwareMap.dcMotor.get("LF");
        LeftBack = hardwareMap.dcMotor.get("LB");
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        Lift = hardwareMap.dcMotor.get("Lift");
        Lift.setPower(0.0);

        GlyphArm1 = hardwareMap.servo.get("GlyphArm1");
        GlyphArm2 = hardwareMap.servo.get("GlyphArm2");
        GlyphArm1.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //MAIN LOOPING PROGRAM
    ////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void loop() {

		/*
         * CONTROLS OVERVIEW:
		 *
		 * GAMEPAD 1 controls the Drive Train
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
		 *      (left bumper) = Loads ball top launcher, down if not pressed
		 *      (right bumper) = Spins launcher motor when pressed
		  *     (right trigger) = reverses laucher motor for position correction
		 */

        //Drive Train Control
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
            RightFront.setPower(right);                // enable turbo mode for the wheels
            RightBack.setPower(right);                 // (up to 100% speed).
            LeftFront.setPower(left);                  //
            LeftBack.setPower(left);                   //

        } else {                                            //  Otherwise, make the wheels go at
            RightFront.setPower(0.5 * right);          // normal speed (up to 50% speed)
            RightBack.setPower(0.5 * right);           //
            LeftFront.setPower(0.5 * left);            //
            LeftBack.setPower(0.5 * left);             //
        }

        //Glyph Arms
        ///////////////////////////////////////////////////////////////
        if(gamepad2.a){
            GlyphArm1.setPosition(0.42);
            GlyphArm2.setPosition(0.42);}
        else{
            GlyphArm1.setPosition(0.7);
            GlyphArm2.setPosition(0.7);}

        //Glyph Lift
        //////////////////////////////////////////////////////////////
        double lift_power = -gamepad2.left_stick_y;
        lift_power = Range.clip(lift_power, -1, 1);
        Lift.setPower(0.5*lift_power);
    }
}