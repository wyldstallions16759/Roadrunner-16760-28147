package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem {

    // Motors for elevating and extending the arm
    private DcMotor ElevatorA = null;
    private DcMotor ElevatorB = null;

    // Use to print to the driver hub
    private Telemetry telemetry;

    // Save targets for elevating and extending the arm
    private int verticalTarget;
    private int horizontalTarget;


    // Constructor
    public ArmSubsystem(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        // Initialize arm motors
        // Reset the motor encoders so they have a value of 0 at startup
        ElevatorA = hwMap.get(DcMotor.class, "ElevatorA");
        ElevatorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElevatorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevatorA.setDirection(DcMotorSimple.Direction.REVERSE);

        ElevatorB = hwMap.get(DcMotor.class, "ElevatorB");
        ElevatorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElevatorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevatorB.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // Set the vertical target position for subsequent armUp and armDown calls
    // Target is remembered until this function is called again
    public void setverticalTarget(int newPosition){
        verticalTarget = newPosition;
    }

    // Set the horizontal target position for subsequent armExtend and armRetract calls
    // Target is remembered until this function is called again

    //------------------------------------------------------------------------------------
    // Move arm up at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setverticalTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean ElevatorAUp(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (ElevatorA.getCurrentPosition() < verticalTarget) {
            ElevatorA.setPower(power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        ElevatorA.setPower(0);
        return true;
    }
    public boolean ElevatorBUp(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (ElevatorB.getCurrentPosition() < verticalTarget) {
            ElevatorB.setPower(power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        ElevatorA.setPower(0);
        return true;
    }
    public boolean ElevatorACatch(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (ElevatorA.getCurrentPosition() < ElevatorB.getCurrentPosition()) {
            ElevatorA.setPower(power+0.02);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        ElevatorA.setPower(0);
        return true;
    }
    public boolean ElevatorBCatch(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (ElevatorB.getCurrentPosition() < ElevatorA.getCurrentPosition()) {
            ElevatorB.setPower(power+0.02);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        ElevatorA.setPower(0);
        return true;
    }
    public boolean verTo(double speed,double target,int tolerance,double power) {
        int positionA = ElevatorA.getCurrentPosition();
        int positionB = ElevatorB.getCurrentPosition();
        int combinedPos = (positionA+positionB)/2;
        telemetry.addData("Arm Position A: ", positionA);
        telemetry.addData("Arm Position B: ", positionB);
        telemetry.update();
        if (Math.abs(target - combinedPos) < tolerance){
            ElevatorA.setPower(0);
            ElevatorB.setPower(0);
            return false;
        }

        if (combinedPos < target) {
            if (Math.abs(positionA - positionB) < 50) {
                ElevatorA.setPower(power);
                ElevatorB.setPower(power);
            } else if (positionA > positionB || !(Math.abs(positionA - positionB) < 50)) {
                ElevatorA.setPower(power);
                ElevatorB.setPower(power + 0.02);
            } else if (positionB > positionA || !(Math.abs(positionA - positionB) < 50)) {
                ElevatorB.setPower(power);
                ElevatorA.setPower(power+0.02);
            }
        } else if (combinedPos > target){
            if (positionA - positionB < 50) {
                ElevatorA.setPower(-power);
                ElevatorB.setPower(-power);
            } else if (positionA > positionB || !(Math.abs(positionA - positionB) < 50)) {
                ElevatorA.setPower(-power);
                ElevatorB.setPower(-power + 0.02);
            } else if (positionB > positionA || !(Math.abs(positionA - positionB) < 50)) {
                ElevatorB.setPower(-power);
                ElevatorA.setPower(-power + 0.02);
            }
        }
        return true;
    }
    //------------------------------------------------------------------------------------
    // Move arm down at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // sethorizontalTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------

}