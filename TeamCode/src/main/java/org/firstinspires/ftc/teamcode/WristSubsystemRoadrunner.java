package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystemRoadrunner{
	private Servo wrist;
	private Servo claw;

	enum WristState{
		UP,
		DOWN
	}
	enum ClawState{
		CLOSED,
		OPEN
	}

	private WristState wristState = WristState.UP;
	private ClawState clawState = ClawState.OPEN;

	private Telemetry telemetry;

	public WristSubsystemRoadrunner(HardwareMap hardwareMap, Telemetry telemetry){
		wrist = hardwareMap.get(Servo.class, "ShoulderPivot");
		claw = hardwareMap.get(Servo.class, "RightFinger");	

		this.telemetry = telemetry;
		wrist.scaleRange(0.35, 0.7);
		claw.scaleRange(0.4, 0.7);
	}

	public class WristUp implements Action{
		@Override
		public boolean run(@NonNull TelemetryPacket packet){
			wristUp_worker();
			return false; // return false as to do nothing. All actual servo movements will be handled by the class that calls this.
		}
	}

	public class WristDown implements Action{
		@Override
		public boolean run(@NonNull TelemetryPacket packet){
			wristDown_worker();
			return false; // return false as to do nothing. All actual servo movements will be handled by the class that calls this.
		}
	}
	
	private void wristUp_worker(){
		// move the wrist to the side
		this.wrist.setPosition(1);
		this.wristState = WristState.UP;
	}
	
	private void wristDown_worker(){
		// move the wrist to a downward position
		this.wrist.setPosition(0);
		this.wristState = WristState.DOWN;
	}

	public class ClawClose implements Action{
		@Override
		public boolean run(@NonNull TelemetryPacket packet){
			clawClose_worker();
			return false; // return false as to do nothing. All actual servo movements will be handled by the class that calls this.
		}
	}
	public class ClawOpen implements Action{
		@Override
		public boolean run(@NonNull TelemetryPacket packet){
			clawOpen_worker();
			return false; // return false as to do nothing. All actual servo movements will be handled by the class that calls this.
		}
	}

	private void clawClose_worker(){
		// move the claw closed
		this.claw.setPosition(1);
		this.clawState = ClawState.CLOSED;
	}

	private void clawOpen_worker(){
		// open the claw
		this.claw.setPosition(0);
		this.clawState = ClawState.OPEN;
	}

	public class ClawToggle implements Action{
		@Override
		public boolean run(@NonNull TelemetryPacket packet){
			toggleClaw_worker();
			return false; // return false as to do nothing. All actual servo movements will be handled by the class that calls this.
		}
	}

	public class WristToggle implements Action{
		@Override
		public boolean run(@NonNull TelemetryPacket packet){
			toggleWrist_worker();
			return false; // return false as to do nothing. All actual servo movements will be handled by the class that calls this.
		}
	}

	private void toggleWrist_worker(){
		// opens the wrist if it is closed.
		// closes the wrist if it is open.

		if (this.wristState == WristState.DOWN){
			this.wristUp_worker();
		}
		else{
			this.wristDown_worker();
		}
	}

	private void toggleClaw_worker(){
		// opens the claw if it is closed.
		// closes the claw if it is open.

		if (this.clawState == ClawState.CLOSED){
			this.clawOpen_worker();
		}
		else{
			this.clawClose_worker();
		}
	}

	//below: Action creator methods

	public Action wristUp(){
		return new WristUp();
	}
	public Action wristDown(){
		return new WristDown();
	}
	public Action clawClose(){
		return new ClawClose();
	}
	public Action clawOpen(){
		return new ClawOpen();
	}
	public Action toggleClaw(){
		return new ClawToggle();
	}
	public Action toggleWrist(){
		return new WristToggle();
	}
}







