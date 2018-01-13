  package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous (name="BHRD_Autonomous1", group="BHRD")

public class BHRD_Autonomous1 extends LinearOpMode {

    DcMotor RightFront;        // These four motors move the wheels.
    DcMotor LeftFront;
    DcMotor RightBack;
    DcMotor LeftBack;
    static final double Counts_Per_Revolution = 1120;    // AndyMark Motor Encoder
    static final double Wheel_Diameter_Inches = 4.0;     // For figuring circumference
    static final double Wheel_Degrees = 360.0;
    static final double Counts_Per_Inch = (Counts_Per_Revolution) /
            (Wheel_Diameter_Inches * 3.1415);
    static final double Counts_Per_Degree = (Counts_Per_Revolution) / (Wheel_Degrees);
    static final double ForwardSpeed = 0.5;
    static final double BackwardSpeed = -0.5;
    static final double TurnSpeed = 0.3;

    ColorSensor Color;
   // int x;
    //int y;

    Servo JewelArm;

    /*public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;*/

    @Override
    public void runOpMode() {

        //initialize motors
        RightFront = hardwareMap.dcMotor.get("RF");
        RightBack = hardwareMap.dcMotor.get("RB");
        LeftFront = hardwareMap.dcMotor.get("LF");
        LeftBack = hardwareMap.dcMotor.get("LB");
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Color = hardwareMap.colorSensor.get("Color");

        JewelArm = hardwareMap.servo.get("JewelArm");
        JewelArm.setPosition(0.0);
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
        // ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters
        //  (cameraMonitorViewId);

        //parameters.vuforiaLicenseKey = "AevcyKb/////AAAAGcNCfeeI70TXjBTxeL/eIsgxIEu9GntEWN5peFjtjnEL3505NSlYhZ5cQf9ZPoYM960MPPf6i2x8/6furBo6kgP+yz25307AP1J12l0mv2bNgmrJ0eW0gLzcCNVJnoffGPrL0p1taZkBBz1C7uoymvmfCqjJwEXIEFqGr1v5269of0RcsAoI3IOftyloTmQDA+zL7hzkz62z1ir1vzwME8w6r23eGteEIfr6VIjeyAOWW2PcPcg88UnYaXQFOKZFE+h2zU/C71y0TPxuXCT0f6aySQoDW4Sw1HLp+CHPKddFp4ZTJIb8QSKwvytLL8ooBGHMFFdsV/6yoqgGmTVa1dbkiQyFOhxvz/zxe6057LEF";
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        //VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicVuMarkTemplate");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        //relicTrackables.activate();

        Jewel(1.0);
        telemetry.addData("Status", "Jewel: Complete");    //
        telemetry.update();

        //IdentifyVuMark();

        DriveForward(36);
        telemetry.addData("Status", "Drive Forward: Complete");    //
        telemetry.update();

        RightTurn(90);
        telemetry.addData("Status", "Right Turn: Complete");    //
        telemetry.update();

        DriveForward(24);
        telemetry.addData("Status", "Drive Forward: Complete");    //
        telemetry.update();

        DriveBackward(5);
        telemetry.addData("Status", "Drive Backward0: Complete");    //
        telemetry.update();

    }


    public void DriveBackward(double inches) {

        int TargetDistance;

        if (opModeIsActive()) {

            //Determine target position
            TargetDistance = RightFront.getCurrentPosition() + (int) (inches * Counts_Per_Inch);
            TargetDistance = RightBack.getCurrentPosition() + (int) (inches * Counts_Per_Inch);
            TargetDistance = LeftFront.getCurrentPosition() + (int) (inches * Counts_Per_Inch);
            TargetDistance = LeftBack.getCurrentPosition() + (int) (inches * Counts_Per_Inch);

            RightFront.setTargetPosition(TargetDistance);
            RightBack.setTargetPosition(TargetDistance);
            LeftFront.setTargetPosition(TargetDistance);
            LeftBack.setTargetPosition(TargetDistance);

            //Turn on run to position
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Turn on motors
            RightFront.setPower(BackwardSpeed);
            RightBack.setPower(BackwardSpeed);
            LeftFront.setPower(BackwardSpeed);
            LeftBack.setPower(BackwardSpeed);

            while (opModeIsActive() && (RightFront.isBusy() &&
                    RightBack.isBusy() &&
                    LeftFront.isBusy() &&
                    LeftBack.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", TargetDistance);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        RightFront.getCurrentPosition(),
                        RightBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(),
                        LeftBack.getCurrentPosition());
                telemetry.update();
            }

            //Stop motion
            RightFront.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            LeftBack.setPower(0);

            sleep(500);
        }
    }

    public void DriveForward(double inches) {

        int TargetDistance;

        if (opModeIsActive()) {

            //Determine target position
            TargetDistance = RightFront.getCurrentPosition() + (int) (inches * Counts_Per_Inch);
            TargetDistance = RightBack.getCurrentPosition() + (int) (inches * Counts_Per_Inch);
            TargetDistance = LeftFront.getCurrentPosition() + (int) (inches * Counts_Per_Inch);
            TargetDistance = LeftBack.getCurrentPosition() + (int) (inches * Counts_Per_Inch);

            RightFront.setTargetPosition(TargetDistance);
            RightBack.setTargetPosition(TargetDistance);
            LeftFront.setTargetPosition(TargetDistance);
            LeftBack.setTargetPosition(TargetDistance);

            //Turn on run to position
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Turn on motors
            RightFront.setPower(ForwardSpeed);
            RightBack.setPower(ForwardSpeed);
            LeftFront.setPower(ForwardSpeed);
            LeftBack.setPower(ForwardSpeed);

            while (opModeIsActive() && (RightFront.isBusy() &&
                    RightBack.isBusy() &&
                    LeftFront.isBusy() &&
                    LeftBack.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", TargetDistance);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        RightFront.getCurrentPosition(),
                        RightBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(),
                        LeftBack.getCurrentPosition());
                telemetry.update();
            }

            //Stop motion
            RightFront.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            LeftBack.setPower(0);

            sleep(500);
        }
    }


    public void Jewel(double servoPosition) {

        JewelArm.setPosition(servoPosition);

        if (Color.red() > 200) {
            DriveForward(6);
            //int x = 6;
        } else if (Color.blue() > 200) {
            DriveBackward(6);
            //int y = 6;
        } else {
            RightFront.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            LeftBack.setPower(0);
        }
        JewelArm.setPosition(0.0);
    }

    public void RightTurn(double degrees) {

        int TargetDegrees;

        if (opModeIsActive()) {

            //Determine target position
            TargetDegrees = RightFront.getCurrentPosition() + (int) (degrees * Counts_Per_Degree);
            TargetDegrees = RightBack.getCurrentPosition() + (int) (degrees * Counts_Per_Degree);
            TargetDegrees = LeftFront.getCurrentPosition() + (int) (degrees * Counts_Per_Degree);
            TargetDegrees = LeftBack.getCurrentPosition() + (int) (degrees * Counts_Per_Degree);

            RightFront.setTargetPosition(TargetDegrees);
            RightBack.setTargetPosition(TargetDegrees);
            LeftFront.setTargetPosition(TargetDegrees);
            LeftBack.setTargetPosition(TargetDegrees);

            //Turn on run to position
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Turn on motors
            RightFront.setPower(TurnSpeed);
            RightBack.setPower(TurnSpeed);
            LeftFront.setPower(-TurnSpeed);
            LeftBack.setPower(-TurnSpeed);

            while (opModeIsActive() && (RightFront.isBusy() &&
                    RightBack.isBusy() &&
                    LeftFront.isBusy() &&
                    LeftBack.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", TargetDegrees);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        RightFront.getCurrentPosition(),
                        RightBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(),
                        LeftBack.getCurrentPosition());
                telemetry.update();
            }

            //Stop motion
            RightFront.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            LeftBack.setPower(0);

            sleep(500);
        }
    }
}

   /* public void IdentifyVuMark() {

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
           /* telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
           /* OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            /*if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
               double rZ = rot.thirdAngle;
            }
        } else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();

    }

    //String format(OpenGLMatrix transformationMatrix) {
        //return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }*/
