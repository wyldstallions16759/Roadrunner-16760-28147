package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="XT1-", group="Linear OpMode")
//@Disabled
public class XT1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor ElevatorA = null;
    private DcMotor ElevatorB = null;
    private PinpointDrive drive;
    private ArmSubsystem arm;

//    //private Servo LeftFinger = null;
//    private Servo RightFinger = null;

    //declare subsystems:

    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        ElevatorA = hardwareMap.get(DcMotor.class, "ElevatorA");
        ElevatorB = hardwareMap.get(DcMotor.class, "ElevatorB");

        //rr
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        arm = new ArmSubsystem(hardwareMap,telemetry);
//        //LeftFinger = hardwareMap.get(Servo.class, "LeftFinger");
//        RightFinger= hardwareMap.get(Servo.class, "RightFinger");
        // create subsystems

        WristSubsystem Wrist = new WristSubsystem(hardwareMap, telemetry);
//        ArmSubsystem arm = new ArmSubsystem(hardwareMap,telemetry);
//        wristSubsystem = new WristSubsystem(hardwareMap, telemetry);
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        ElevatorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RightFinger.scaleRange(0.4,0.7);

        ElevatorA.setDirection(DcMotor.Direction.FORWARD);
        ElevatorB.setDirection(DcMotor.Direction.REVERSE);
//        RightFinger.setDirection(Servo.Direction.FORWARD);
        ElevatorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElevatorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        boolean oldWristButton = false;
        float oldClawButton = 0;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            boolean in = gamepad2.a;
            boolean out = gamepad2.y;
            float up = gamepad2.right_stick_y;
            float claw_in = gamepad2.right_trigger;
            float claw_out = gamepad2.left_trigger;
            float claw_toggle = gamepad2.right_trigger;
            boolean wrist_toggle = gamepad2.b;
            boolean release_slightly_claw = gamepad2.left_bumper;
            boolean slow_the_flip_down = gamepad1.right_bumper;
            float claw_left_val = 0;
            float claw_right_val = 0;
            boolean preset_specimen = gamepad2.right_bumper;
            boolean onoroff_Specimen = false;
            boolean reset_encoders = gamepad2.x;
            boolean SUB = gamepad2.dpad_left;
            boolean OZ = gamepad2.dpad_right;
            double dumb = 13.9;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (slow_the_flip_down) {
                max *= 5;
            }

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels

            if (up > 0.05) {
                arm.verTo(0.1,up * 2000,20,0.1);
            } else {

            }

            // Wrist Subsystem calls:

            if (claw_toggle > 0.7 && !(oldClawButton > 0.7)) {
                Wrist.toggleClaw();
            }
            if (wrist_toggle && !oldWristButton) {
                Wrist.toggleWrist();
            }
//            if (SUB) {
//                SUBMERSIBLE = new Pose2D(DistanceUnit.INCH,-29 ,dumb += 2,AngleUnit.DEGREES,0);
//                pinpoint.driveTo(SUBMERSIBLE,0.3,0);
//            } else {
//                leftFrontDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//            }
//            if (OZ) {
//                pinpoint.driveTo(OBSERVATION,0.3,0);
//            } else {
//                leftFrontDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//            }
            oldWristButton = wrist_toggle;
            oldClawButton = claw_toggle;

            // Show the elapsed game time and wheel power.
            PoseVelocity2d pose = drive.updatePoseEstimate();
            telemetry.addData("PoseX", pose.component1().x);
            telemetry.addData("PoseY", pose.component1().y);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
//l
    }

    public boolean Extend(double inches, double speed) {

        // Determine new target position. Use the current position of the LeftFrontDrive
        int currentTarget = (int) (inches * (452.53 / (4 * 3.145)));


        // If motors are still busy, haven't reached target
//        if (LeftFrontDrive.isBusy() && LeftFrontDrive.isBusy() &&
//                LeftFrontDrive.isBusy() && LeftFrontDrive.isBusy()) {
//            return false;
//        }

        //LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return true;
    }
}