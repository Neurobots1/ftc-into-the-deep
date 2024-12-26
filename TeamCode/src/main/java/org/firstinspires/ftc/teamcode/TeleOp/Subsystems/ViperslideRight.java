package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ViperslideRight extends OpMode {

    private PIDController controller;

// Here we set the proportional , intergral , dampener and feedfoward constants
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
// Targer position
    public static int target = 0;

// Number of ticks per revolution
    private final double ticks_in_degree = 537.7 / 360;
//Motors
    private DcMotorEx slidemotorright;
    private DcMotorEx slidemotorleft;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void loop() {

            controller.setPID(p, i, d);
            int slidePosRight = slidemotorright.getCurrentPosition();
            double pidRight = controller.calculate(slidePosRight, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double powerRight = pidRight + ff;

            slidemotorleft.setPower(powerRight);

            telemetry.addData("posRight", slidePosRight);
            telemetry.addData("target", target);
            telemetry.update();



            if (gamepad1.y) {
                target = 2950;
            }
            if (gamepad1.a){
                target = 0;
            }

            if (gamepad1.x){
                target = 900;
            }

            if (gamepad1.b){
                target = 1400;
            }
        }
    }