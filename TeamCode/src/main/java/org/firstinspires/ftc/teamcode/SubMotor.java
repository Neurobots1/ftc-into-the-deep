package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SubMotor{
    public PIDController controller;


    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = -0.1;

    public static int target = 0;

    public final double ticks_in_degree = 384.5 / 180.0;

    public DcMotorEx slidemotorright;
    public DcMotorEx slidemotorleft;

    public void init() {
    }


    public void loop() {

            controller.setPID(p, i, d);
            int slidePos = slidemotorright.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            slidemotorright.setPower(power);
            slidemotorleft.setPower(-power);
            telemetry.addData("pos", slidePos);
            telemetry.addData("target", target);
            telemetry.update();

        }
    }
