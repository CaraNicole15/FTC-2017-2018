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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="AutoTimeGyro", group="ahert" )
//@Disabled
public class AutoTimeGyro extends LinearOpMode {


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

    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device



    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV ) /
                                                       (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double Launcher_SPEED = 1.0;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable



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

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");



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

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 38.0, 0.0);    // Drive FWD 38 inches
        encoderLauncher(Launcher_SPEED,1,0.5); //launch ball #1
        LoadBall(1); //load ball #2
        encoderLauncher(Launcher_SPEED,1,0.5); //launch ball #2
        gyroDrive(DRIVE_SPEED,20.0, 0.0);    // Drive REV 48 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();}


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

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = motorLeftFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = motorLeftBack.getCurrentPosition() + moveCounts;
            newRightFrontTarget = motorRightFront.getCurrentPosition() + moveCounts;
            newRightBackTarget = motorRightBack.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorLeftBack.setTargetPosition(newLeftBackTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);
            motorRightBack.setTargetPosition(newRightBackTarget);


            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorLeftFront.setPower(speed);
            motorLeftBack.setPower(speed);
            motorRightFront.setPower(speed);
            motorRightBack.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                          motorLeftFront.isBusy() &&
                            motorLeftBack.isBusy() &&
                            motorRightFront.isBusy() &&
                            motorRightBack.isBusy())  {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motorLeftFront.setPower(leftSpeed);
                motorLeftBack.setPower(leftSpeed);
                motorRightFront.setPower(rightSpeed);
                motorRightBack.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftFrontTarget, newLeftBackTarget,
                        newRightFrontTarget,newRightBackTarget);
                telemetry.addData("Actual",  "%7d:%7d",      motorLeftFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition(), motorRightFront.getCurrentPosition(),
                        motorRightBack.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motorLeftFront.setPower(leftSpeed);
        motorLeftBack.setPower(rightSpeed);
        motorRightFront.setPower(leftSpeed);
        motorRightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}


