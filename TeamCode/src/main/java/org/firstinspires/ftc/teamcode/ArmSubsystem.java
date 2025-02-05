package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem {

    // Motors for elevating and extending the arm
    private DcMotor vertical = null;
    private DcMotor horizontal = null;

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
        vertical = hwMap.get(DcMotor.class, "Verticle");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontal = hwMap.get(DcMotor.class, "horizontal");
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set the vertical target position for subsequent armUp and armDown calls
    // Target is remembered until this function is called again
    public void setverticalTarget(int newPosition){
        verticalTarget = newPosition;
    }

    // Set the horizontal target position for subsequent armExtend and armRetract calls
    // Target is remembered until this function is called again
    public void sethorizontalTarget(int newPosition){
        horizontalTarget = newPosition;
    }

    //------------------------------------------------------------------------------------
    // Move arm up at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setverticalTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armUp(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (vertical.getCurrentPosition() > verticalTarget) {
            vertical.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        vertical.setPower(0);
        return true;
    }
    public boolean horTo(double speed,int target,int tolerance) {
        int position = vertical.getCurrentPosition();
        telemetry.addData("Arm Position: ", position);
        telemetry.update();
        if (Math.abs(target - position) < tolerance){
            vertical.setPower(0);
            return false;
        }

        if (position < target) {
            vertical.setPower(speed);
        } else {
            vertical.setPower(-speed);
        }
        return true;
    }
    public boolean verTo(double speed,int target,int tolerance) {
        int position = horizontal.getCurrentPosition();
        telemetry.addData("Arm Position: ", position);
        telemetry.update();
        if (Math.abs(target - position) < tolerance){
            vertical.setPower(0);
            return false;
        }

        if (position < target) {
            vertical.setPower(speed);
        } else {
            vertical.setPower(-speed);
        }
        return true;
    }
    //------------------------------------------------------------------------------------
    // Move arm down at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // sethorizontalTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armDown(double power) {
        // Target not yet reached
        if (vertical.getCurrentPosition() < verticalTarget) {
            // Due to the way the arm motor is oriented, positive speed moves it down
            vertical.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        vertical.setPower(0);
        return true;
    }

    //------------------------------------------------------------------------------------
    // Extend arm out at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // sethorizontalTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armExtend(double power) {
        // Target not yet reached
        if (horizontal.getCurrentPosition() < horizontalTarget){
            telemetry.addData("reached position", 1);
            // Positive speed extends the arm
            horizontal.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        horizontal.setPower(0);
        return true;
    }

    //------------------------------------------------------------------------------------
    // Retract arm at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // sethorizontalTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armRetract(double power) {
        // Target not yet reached
        if (horizontal.getCurrentPosition() > horizontalTarget){
            telemetry.addData("reached position", 1);
            // Negative speed retracts the arm
            horizontal.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        horizontal.setPower(0);
        return true;
    }

    //-----------------------------
    // Getter functions
    //-----------------------------
    public int getCurrElevPosition () {
        return vertical.getCurrentPosition();
    }

    public int getElevTargetPosition () {
        return verticalTarget;
    }

    public int getCurrExtPosition () {
        return horizontal.getCurrentPosition();
    }

    public int getExtTargetPosition () {
        return horizontalTarget;
    }
}