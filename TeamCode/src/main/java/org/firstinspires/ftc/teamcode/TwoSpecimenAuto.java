package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Two Specimen Auto", group="!TEST!")
public class TwoSpecimenAuto extends LinearOpMode {
    // subsystems
    private PinpointDrive drive;
    private WristSubsystemRoadrunner wrist;
    private ArmSubsystemRoadrunner arm;

    // constants
    private Pose2d START_POSE = new Pose2d(0,0,0);

    private final int ARM_ROT_SUBMERSIBLE_CLIP = 1700;
    private final int ARM_ROT_SUBMERSIBLE_CLIP_FINAL = 1700; // above two might need to be adjusted
    private final int ARM_DOWN_TO_GRAB = 4500; // WILL need to be determined
    private final int ARM_DOWN_GRABBING = 5000; // Difference from above: one is slightly above ground

    private final int EXTENSION_SUBMERSIBLE = 7500;
    private final int EXTENSION_CLIP = 1500;

    @Override
    public void runOpMode(){
        // initialize subsystems
        drive = new PinpointDrive(hardwareMap, START_POSE);
        wrist = new WristSubsystemRoadrunner(hardwareMap, telemetry);
        arm = new ArmSubsystemRoadrunner(hardwareMap, telemetry);

        // create path

        // STEPS:
        // Drive to submersible, raise arm, extend arm, close claw
        Action step1 = new ParallelAction(
                arm.armTo(1, ARM_ROT_SUBMERSIBLE_CLIP),//or, only move part-way...
                arm.extendTo(1, EXTENSION_SUBMERSIBLE),
                wrist.clawClose(),
                wrist.wristDown(),
                drive.actionBuilder(START_POSE)
                        .lineToY(20)
                        .lineToX(28)
                        .build()
        );
        // finish raising arm
        Action step2 = arm.armTo(1, ARM_ROT_SUBMERSIBLE_CLIP_FINAL);
        // dextend arm
        Action step3 = arm.extendTo(1, EXTENSION_CLIP);
        // open claw
        Action step4 = wrist.clawOpen();
        // drive to push specimen, lower arm, finish lower extend
        Action step5 = new ParallelAction(
                arm.extendTo(1, 0),
                arm.armTo(1, 0),
                drive.actionBuilder(START_POSE)
                        .lineToX(24)
                        .lineToY(-12)
                        .lineToX(52)
                        .lineToY(-19)
                        .turnTo(Math.toRadians(180))
                        .lineToX(10)
                        .lineToX(20) /// might need to be changed
                        .build()
        );
        // in parallel, turn around, wait........ (for human player)
        Action step6 = new ParallelAction(
                drive.actionBuilder(START_POSE)
                        .lineToY(-39)
                        .build(),
                new Pause(5000), // wait 5 seconds for human player
                arm.armTo(1, ARM_DOWN_TO_GRAB),
                wrist.clawOpen()
        );
        // pick up (lower arm, close claw
        Action step7 = new ParallelAction(
                drive.actionBuilder(START_POSE)
                        .lineToX(14)
                        .build(),
                arm.armTo(1, ARM_DOWN_GRABBING)
        );

        //close claw
        Action step8 = wrist.clawClose();

        // drive to submersible, raise arm, extend arm
        Action step9 = new ParallelAction(
                drive.actionBuilder(START_POSE)
                        .lineToY(18)
                        .turnTo(0)
                        .lineToX(29) // sideways wrist; a little closer
                        .build(),
                arm.armTo(1, ARM_ROT_SUBMERSIBLE_CLIP),
                arm.extendTo(1,EXTENSION_SUBMERSIBLE),
                wrist.wristUp()
        );

        // finish arm
        Action step10 = arm.armTo(1, ARM_ROT_SUBMERSIBLE_CLIP_FINAL);

        // pull down (dextend) arm
        Action step11 = arm.extendTo(1, EXTENSION_CLIP);

        // Release
        Action step12 = wrist.clawOpen();

        // drive to obs.
        Action step13 = drive.actionBuilder(START_POSE)
                .lineToX(2)
                .lineToY(-27)
                .build();

        Action COMPILED_ACTIONS = new ParallelAction(
                step1,
                step2,
                step3,
                step4,
                step5,
                step6,
                step7,
                step8,
                step9,
                step10,
                step11,
                step12,
                step13,
                new Pause(30000)
        );

        waitForStart();

        while (opModeIsActive()){
            Actions.runBlocking(COMPILED_ACTIONS);
        }


    }
}
