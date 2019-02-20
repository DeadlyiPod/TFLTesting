package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by johnb on 10/23/2018.
 */


@TeleOp(name="DriveTrain", group="TeleOp")
public class DriveTrainTeleOp extends LinearOpMode{

    //Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftInnerDrive = null;
    private DcMotor leftOuterDrive = null;

    private DcMotor rightInnerDrive = null;
    private DcMotor rightOuterDrive = null;

    private DcMotor slideMotor = null;
    private DcMotor rotateMotor = null;
    private DcMotor hangMotor = null;
    private DcMotor liftMotor = null;

    private CRServo intake = null;
    private Servo outTake = null;
    private Servo depotDrop = null;

    //TeleOp Driving Settings
    double speedScale;
    double speedScaleSlow = 0.6;
    double speedScaleFast = 1;
    int outTakeIndex = 0;
    double outTakePos0 = 1;
    double outTakePos1 = 0.85;
    double outTakePos2 = 0.67;

    int hangInitPos;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Initialize hardware variables
        leftInnerDrive = hardwareMap.get(DcMotor.class, "leftInnerDrive");
        leftOuterDrive = hardwareMap.get(DcMotor.class, "leftOuterDrive");

        rightInnerDrive = hardwareMap.get(DcMotor.class, "rightInnerDrive");
        rightOuterDrive = hardwareMap.get(DcMotor.class, "rightOuterDrive");

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        rotateMotor = hardwareMap.get(DcMotor.class, "rotateMotor");
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");


        intake = hardwareMap.get(CRServo.class, "Intake");
        outTake = hardwareMap.get(Servo.class, "outTake");
        depotDrop = hardwareMap.get(Servo.class, "depotDrop");

        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motor Polarities
        leftInnerDrive.setDirection(DcMotor.Direction.REVERSE);
        leftOuterDrive.setDirection(DcMotor.Direction.REVERSE);

        rightInnerDrive.setDirection(DcMotor.Direction.FORWARD);
        rightOuterDrive.setDirection(DcMotor.Direction.FORWARD);

        if(outTake.getController() instanceof ServoControllerEx){
            ServoControllerEx theControl = (ServoControllerEx) outTake.getController();

            int thePort = outTake.getPortNumber();
            PwmControl.PwmRange theRange = new PwmControl.PwmRange(500,2500);
            theControl.setServoPwmRange(thePort,theRange);
        }

        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //wait for driver to press PLAY
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");     telemetry.update(); }
        runtime.reset();


        hangInitPos = hangMotor.getCurrentPosition();

        //run until driver presses STOP
        while (opModeIsActive()) {

            //DRIVING
            {
                //Drive power variables
                double leftInnerPower;
                double leftOuterPower;
                double rightInnerPower;
                double rightOuterPower;

                double liftMotorPower;
                double slidePower;
                double rotatePower;
                double hangMotorPower;
                double intakePower;


                //Tank mode controls

                //Take sum of values of joysticks and triggers, then clip them to 1.00
                leftInnerPower = Range.clip(-gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger, -1.0, 1.0);
                leftOuterPower = Range.clip(-gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger, -1.0, 1.0);

                rightInnerPower = Range.clip(-gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger, -1.0, 1.0);
                rightOuterPower = Range.clip(-gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger, -1.0, 1.0);


                //Scale power value for more controllable driving
                if (gamepad1.left_bumper) {
                    speedScale = speedScaleFast;
                } else {
                    speedScale = speedScaleSlow;
                }

                leftInnerPower = Range.scale(leftInnerPower, -1.00, 1.00, -speedScale, speedScale);
                leftOuterPower = Range.scale(leftOuterPower, -1.00, 1.00, -speedScale, speedScale);

                rightInnerPower = Range.scale(rightInnerPower, -1.00, 1.00, -speedScale, speedScale);
                rightOuterPower = Range.scale(rightOuterPower, -1.00, 1.00, -speedScale, speedScale);
                if (gamepad2.y && hangMotor.getCurrentPosition() < (0)) {
                    hangMotorPower = 1;
                }else if (gamepad2.a && hangMotor.getCurrentPosition() > (-10800)) {
                    hangMotorPower = -1;
                } else if (gamepad2.y && gamepad2.x) {
                    hangMotorPower = 1;
                } else if (gamepad2.a && gamepad2.x) {
                    hangMotorPower = -1;
                }else {
                    hangMotorPower = 0;
                }




                //intakePower = Range.scale(-gamepad2.left_trigger + gamepad2.right_trigger,-1,1,-0.3,0.3);

                if (gamepad2.left_trigger > 0) {
                    intake.setPower(1);
                } else if (gamepad2.right_trigger > 0) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }
                slidePower = Range.scale(gamepad2.left_stick_y, -1.00, 1.00, -0.85, 0.70);
                rotatePower = Range.scale(-gamepad2.right_stick_y, -1.00, 1.00, -0.65, 0.80);

                if (gamepad2.right_bumper) {
                    liftMotorPower = 1;
                } else if (gamepad2.left_bumper) {
                    liftMotorPower = -1;
                } else {
                    liftMotorPower = 0;
                }

                if (gamepad2.b && !gamepad2.start) {
                    depotDrop.setPosition(0);
                }
                if (gamepad2.x) {
                    depotDrop.setPosition(1);
                }

                if (gamepad2.dpad_up && outTakeIndex == 0) {
                    outTake.setPosition(outTakePos1);
                    outTakeIndex = 1;
                    sleep(200);
                } else if (gamepad2.dpad_up && outTakeIndex == 1) {
                    outTake.setPosition(outTakePos2);
                    outTakeIndex = 2;
                    sleep(200);
                } else if (gamepad2.dpad_down && outTakeIndex == 2) {
                    outTake.setPosition(outTakePos1);
                    outTakeIndex = 1;
                    sleep(200);
                } else if (gamepad2.dpad_down && outTakeIndex == 1) {
                    outTake.setPosition(outTakePos0);
                    outTakeIndex = 0;
                    sleep(200);
                }


                //Set Power
                leftInnerDrive.setPower(leftInnerPower);
                leftOuterDrive.setPower(leftOuterPower);

                rightInnerDrive.setPower(rightInnerPower);
                rightOuterDrive.setPower(rightOuterPower);

                rotateMotor.setPower(rotatePower);
                slideMotor.setPower(slidePower);
                liftMotor.setPower(liftMotorPower);
                hangMotor.setPower(hangMotorPower);

                if(runtime.seconds() == 90){
                    depotDrop.setPosition(1);
                }


                //Runtime, and Power Variables being sent to motor
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Drivetrain Encoder Values", "\n%s%d\n%s%d\n%s%d\n%s%d", "Left Inner Drive: ", leftInnerDrive.getCurrentPosition(), "Right Inner Drive: ", rightInnerDrive.getCurrentPosition(), "Left Outer Drive: ", leftOuterDrive.getCurrentPosition(), "Right Outer Drive:", rightOuterDrive.getCurrentPosition());
                telemetry.addData("Hanging Motor Encoder: ", hangMotor.getCurrentPosition());
                telemetry.update();

            } //Driving END

        }
    }

}