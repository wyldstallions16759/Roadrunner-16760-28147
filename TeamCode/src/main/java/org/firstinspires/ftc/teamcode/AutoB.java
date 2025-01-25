package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto B the Curry")
public class AutoB extends LinearOpMode {
    // systems
    private PinpointDrive drive;
    private ArmSubsystemRoadrunner armSubsystem;
    private WristSubsystem wrist;

    // pose2ds
    private Pose2d startPose = new Pose2d(0, 0, 0);
    private Pose2d waypointA = new Pose2d(15, -53.6, 0);

    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, startPose);
        armSubsystem = new ArmSubsystemRoadrunner(hardwareMap, telemetry);
        wrist = new WristSubsystem(hardwareMap,telemetry);

        SequentialAction action = new SequentialAction(
                // run this, in parallel
                new ParallelAction(

                        drive.actionBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(28,13),0.5)
                        .build(),
                        armSubsystem.armTo(1, 1800)
//                        armSubsystem.extendTo(1,2000)
                ),
//                new SequentialAction(
//                    armSubsystem.extendTo(1,4000)
//                ),
                new ParallelAction(

                drive.actionBuilder(new Pose2d(28,13,0))
                        .strafeToLinearHeading(new Vector2d(-5,30),0)
                        .build(),
                armSubsystem.armTo(1, 0)
                )
                // then run this.
//                new ParallelAction(
//                        armSubsystem.armTo(1, 0),
//                        drive.actionBuilder(startPose)
//                                .strafeTo(new Vector2d(-144,0))
//                                .build()
                );

        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(action);
        }
    }
}
