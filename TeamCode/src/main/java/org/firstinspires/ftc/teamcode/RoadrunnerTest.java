package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Roadrunner Test")
public class RoadrunnerTest extends LinearOpMode {
    // systems
    private PinpointDrive drive;
    private ArmSubsystemRoadrunner armSubsystem;

    // pose2ds
    private Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, startPose);
        armSubsystem = new ArmSubsystemRoadrunner(hardwareMap, telemetry);

        SequentialAction action = new SequentialAction(
                // run this, in parallel
                        new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(144,0))
                        .build(),
                    armSubsystem.armTo(1, 3000)
                ),
                // then run this.
                new ParallelAction(
                        armSubsystem.armTo(1, 0),
                        drive.actionBuilder(startPose)
                                .strafeTo(new Vector2d(-144,0))
                                .build()
                ));

        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(action);
        }
    }
}
