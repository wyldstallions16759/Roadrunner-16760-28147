package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Roadrunner Test")
public class RoadrunnerTestAuto extends LinearOpMode {
    private PinpointDrive drive;
    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        drive.pinpoint.setPositionRR(new Pose2d(0,0,0));
        waitForStart();
        while (opModeIsActive()){
            drive.updatePoseEstimate();
            telemetry.addLine("Roadrunner Pose: ");
            telemetry.addData("x",drive.pose.position.x);
            telemetry.addData("y",drive.pose.position.y);
            telemetry.addData("heading",drive.pose.heading.toDouble());
            telemetry.addData("heading",drive.pose.heading.toDouble()*180/Math.PI);

            telemetry.addLine("Pinpoint Pose: ");
            Pose2D pos = drive.pinpoint.getPosition();
            telemetry.addData("x",pos.getX(DistanceUnit.INCH));
            telemetry.addData("y",pos.getY(DistanceUnit.INCH));
            telemetry.addData("heading",pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("heading",pos.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }
            /*Actions.runBlocking(
                    new SequentialAction(
                        drive.actionBuilder(new Pose2d(0,0,0))
                                .turn(Math.PI/2)
                                .build()
                    )

            );
            telemetry.addData("x",drive.pose.position.x);
            telemetry.addData("y",drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading.toDouble());
            telemetry.addLine("CAPTION");
            telemetry.update();
            sleep(10000);*/
    }
}