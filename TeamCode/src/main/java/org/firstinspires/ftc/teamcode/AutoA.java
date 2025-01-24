package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name="Auto16760")
//@Disabled

public class AutoA extends LinearOpMode {

    // Auto State Machine
    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SUBMERSIBLE,
        RETRACT_ARM,
        RELEASE_SPECIMEN,
        DRIVE_TO_GP_1A,
        DRIVE_TO_GP_1B,
        DRIVE_TO_GP_1C,
        DRIVE_TO_GP_1D,
        DRIVE_TO_OBSERVATION_ZONE,
        END
    }

    StateMachine stateMachine;
    ArmSubsystem arm;
    WristSubsystem wrist;

    double heading;

    //-----------------------------------------------------------
    // Field poses and arm positions for each state
    //-----------------------------------------------------------

    // ----- State: DRIVE_TO_SUBMERSIBLE -----
    static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, -28, 13.6, AngleUnit.DEGREES, 0);
    static final int ARM_ELEV_PLACE_SPECIMEN = -900;
    static final int ARM_EXTEND_PLACE_SPECIMEN = 7500;

    // ----- State: PLACE_SPECIMEN -----
    static final int ARM_EXTEND_RELEASE_SPECIMEN = 4500;

    // ----- State: RELEASE_SPECIMEN -----
    // Only operation in this state is to toggle claw, so no need to wait for it to complete

    // ----- State: DRIVE_TO_GP_1A (backup robot, move right past submersible, turn 180)
    static final Pose2D GP1_POSA = new Pose2D(DistanceUnit.INCH, -20, 53.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_GP_1B (go forward)
    static final Pose2D GP1_POSB = new Pose2D(DistanceUnit.INCH, -62, 53.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_GP_1C (move right)
    static final Pose2D GP1_POSC = new Pose2D(DistanceUnit.INCH, -46, 59.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_GP_1D (push game piece to observation zone)
    static final Pose2D GP1_POSD = new Pose2D(DistanceUnit.INCH, -6, 59.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_OBSERVATION_ZONE
    static final Pose2D OBSERVATION_ZONE = new Pose2D(DistanceUnit.INCH, -5, 70, AngleUnit.DEGREES, 90);
    static final int ARM_ELEV_START_POS = 0;
    static final int ARM_EXTEND_START_POS = 0;

    // -----------------------------------
    // Drive and Arm Speeds
    // -----------------------------------
    static final double ARM_ELEVATION_POWER = 1;
    static final double ARM_EXTENSION_POWER = 1;
    static final double DRIVE_SPEED = 0.45;
    Trajectory driveToSubmersible;


    @Override
    public void runOpMode() {

        // Initialize Subsystems
        Pose2d beginPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        wrist = new WristSubsystem(hardwareMap, telemetry);

        // Initialize state machine
        stateMachine = StateMachine.WAITING_FOR_START;

        // Initialize wrist to starting position
        wrist.wristUp();
        wrist.clawClose();
        //trajectory builder driver
        /*driveToSubmersible = */drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(10,5))
                .build();
        // Wait for Autonomous to start
        waitForStart();
        resetRuntime();

        // Main loop for Auto
        while (opModeIsActive()) {

            // IMPORTANT: odometry needs to be updated every time through the loop
            drive.updatePoseEstimate();

            // Send debug info to driver hub
            displayDebugInfo();
            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
            //----------------------------------------------------------
            // State: WAITING_FOR_START
            // Starting state. Move wrist down into position for placing specimen
            //----------------------------------------------------------
            if (stateMachine == StateMachine.WAITING_FOR_START) {
                wrist.wristDown();
                stateMachine = StateMachine.DRIVE_TO_SUBMERSIBLE;
            }
            else if (stateMachine == StateMachine.DRIVE_TO_SUBMERSIBLE) {
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                //.splineTo(new Pose(0, 60), Math.PI)
                                .build());
            }
            //----------------------------------------------------------
            // State: DRIVE_TO_SUBMERSIBLE
            // Actions: Drive to the front of the submersible and elevate and extend arm
            // Next State: DRIVE_TO_SUBMERSIBLE
            //---------------------------------------

            //----------------------------------------------------------a
            // State: END
            // Actions: Done with auto routine
            //----------------------------------------------------------
            else if (stateMachine == StateMachine.END) {

            }
        }
    }

    private void displayDebugInfo() {
        telemetry.addData("State: ", stateMachine);
        telemetry.addData("ArmElevCurrPosition", arm.getCurrElevPosition());
        telemetry.addData("ArmElevTargetPosition", arm.getElevTargetPosition());
        telemetry.addData("ArmExtCurrPosition", arm.getCurrExtPosition());
        telemetry.addData("ArmExtTargetPosition", arm.getExtTargetPosition());
        telemetry.update();
    }
}