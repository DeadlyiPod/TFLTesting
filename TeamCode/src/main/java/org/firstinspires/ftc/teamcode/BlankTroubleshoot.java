package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by johnb on 10/23/2018.
 */


@TeleOp(name="TrblSht", group="TeleOp")
public class BlankTroubleshoot extends LinearOpMode{

    //Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        //wait for driver to press PLAY
        waitForStart();
        runtime.reset();

        //run until driver presses STOP
        while (opModeIsActive()) {

            telemetry.update();
            }

        }

    }