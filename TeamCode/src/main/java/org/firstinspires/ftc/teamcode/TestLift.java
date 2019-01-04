package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Guard on 1/3/2019.
 */
@TeleOp(name = "Test Lift", group = "Testing")
public class TestLift extends LinearOpMode {

    public DcMotor hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");

    @Override
    public void runOpMode(){
        while(opModeIsActive()){
            double hangMotorPower;

            hangMotorPower = Range.scale(gamepad1.right_stick_y,-1,1,-1,1);
            hangMotor.setPower(hangMotorPower);
        }
    }
}
