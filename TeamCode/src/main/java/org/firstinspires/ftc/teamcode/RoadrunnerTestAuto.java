package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Roadrunner Test")
public class RoadrunnerTestAuto extends LinearOpMode {
    private PinpointDrive drive;
    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        waitForStart();

        while (opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                        drive.actionBuilder(new Pose2d(0,0,0))
                                .splineTo(new Vector2d(24, 24), Math.PI/2)
                                .build()
                    )

            );
            telemetry.addData("x",drive.pose.position.x);
            telemetry.addData("y",drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading.real);
            telemetry.addLine("CAPTION");
            telemetry.update();
            sleep(10000);
        }
    }
}