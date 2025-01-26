package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpMode.Subsystems.IntakeServos;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous
public class AutonomousRoadRunner extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Initialize the Servo objects
        Servo intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        Servo intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");

        // Pass the Servo objects to the IntakeServos constructor
        IntakeServos intakeServos = new IntakeServos(intakeServoRight, intakeServoLeft);

        // Define the initial robot pose
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Define the trajectory sequence
        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        // Closeout trajectory
        Action trajectoryActionCloseOut = tab.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        telemetry.addLine("Ready to start. Press Play to begin.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Execute the autonomous sequence
        Actions.runBlocking(
                new ParallelAction(
                        tab.build(),                                  // Execute the main trajectory
                        Actions.run(() -> intakeServos.intakePosition()),  // Move intake servos to intake position
                        Actions.waitSeconds(1),                            // Wait for servo movement
                        Actions.run(() -> intakeServos.transferPosition()), // Move intake servos to transfer position
                        Actions.waitSeconds(1),                            // Wait for servo movement
                        trajectoryActionCloseOut                          // Execute the closeout trajectory
                )
        );
    }
}
