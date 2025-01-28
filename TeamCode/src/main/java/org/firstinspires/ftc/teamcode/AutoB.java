package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoBTwoSpec")
public class AutoB extends LinearOpMode {
    // systems
    private PinpointDrive drive;
    private ArmSubsystemRoadrunner armSubsystem;
    private WristSubsystemRoadrunner wrist;

    // pose2ds
    private static Pose2d startPose = new Pose2d(0, 0, 0);
    private static int ArmSub = 1800;


    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, startPose);
        armSubsystem = new ArmSubsystemRoadrunner(hardwareMap, telemetry);
        wrist = new WristSubsystemRoadrunner(hardwareMap,telemetry);

        SequentialAction action = new SequentialAction(
                // run this, in parallel
                new ParallelAction(

                        wrist.wristDown(),
                        wrist.clawClose(),
                        drive.actionBuilder(startPose)
                                .strafeToConstantHeading(new Vector2d(2,3))
                        .strafeToConstantHeading(new Vector2d(11,1))

                        .build(),
                        armSubsystem.extendTo(1, 7000),
                        armSubsystem.armTo(1, 1700)
                ),
                new SequentialAction(
                        armSubsystem.armTo(1, 2000),
                        armSubsystem.extendTo(1,2000), // decrease this distance. 2000 is not enough
                        wrist.clawOpen()
                )
//                new ParallelAction(
//
//                drive.actionBuilder(drive.pinpoint.getPositionRR())
//                        .splineToConstantHeading(Waypoints.SAMPLE_COLLECT_STEP1,0.5)
//                        .splineToLinearHeading(Waypoints.SAMPLE_COLLECT_STEP2,0)
//                        .splineToLinearHeading(Waypoints.SAMPLE_COLLECT_STEP3,0.5)
//                        .splineToLinearHeading(Waypoints.SAMPLE_COLLECT_STEP4,0)
//                        .build(),
//                armSubsystem.armTo(1, 0),
//                armSubsystem.extendTo(1,0)
//                ),
//                new SequentialAction(
//
//                        drive.actionBuilder(new Pose2d(28,13,0))
//                                .strafeToLinearHeading(new Vector2d(-5,-30),0)
//                                .build()
//
//                )
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
