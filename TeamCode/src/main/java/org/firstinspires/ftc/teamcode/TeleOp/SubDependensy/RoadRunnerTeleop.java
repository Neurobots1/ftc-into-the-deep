package org.firstinspires.ftc.teamcode.TeleOp.SubDependensy;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp
public class RoadRunnerTeleop extends OpMode {
    private double SLOW_DOWN_FACTOR = 0.5;

    @Override
    public void init() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //Orientation
    }
    @Override
    public void loop()

            double forward = -gamepad1.left_stick_y * SLOW_DOWN_FACTOR;
            double right = -gamepad1.left_stick_x * SLOW_DOWN_FACTOR;
            double rotate = -gamepad1.right_stick_x * SLOW_DOWN_FACTOR;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, right ), rotate));

            drive.updatePoseEstimate();

            if(gamepad1.start) {SLOW_DOWN_FACTOR = SLOW_DOWN_FACTOR + .1;}
            if(gamepad1.back) {SLOW_DOWN_FACTOR = SLOW_DOWN_FACTOR - .1;}

        }
    }

}

