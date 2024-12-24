package org.firstinspires.ftc.teamcode.TeleOp.SubDependensy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp
public class test_intake extends LinearOpMode {

    public DcMotorEx slidemotorright;
    public DcMotorEx slidemotorleft;


    public void runOpMode(){

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        boolean intakeToggle = false;
        //boolean poignettoggle = true;
        boolean pinceToggle = false;


        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorleft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");


        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor intake = hardwareMap.dcMotor.get("slidemotorleft");


        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (gamepad1.a){
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.y){
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }



        }
    }
}


