package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Driver-controlled OpMode for the goBILDA StarterBot (team 5243).
 *
 * Controls (gamepad1, same as the goBILDA example):
 *   left stick         drive forward/back + strafe
 *   right stick X      turn
 *   right trigger      intake in (proportional)
 *   left trigger       intake out
 *   right bumper       hold to spin up launcher; feeds automatically once up to speed
 *   left bumper        hold for slow mode (precise lining up)
 *   Y                  toggle field-centric driving
 *   back               reset heading (face away from the driver when you press it)
 */
@TeleOp(name = "5243 StarterBot TeleOp", group = "5243")
public class StarterBotTeleOp extends LinearOpMode {
    public static double SLOW_MODE_SCALE = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StarterBotMechanisms mech = new StarterBotMechanisms(hardwareMap);

        // Pick up where autonomous left off, so field-centric starts out pointed the right way
        drive.setPoseEstimate(PoseStorage.currentPose);

        boolean fieldCentric = false;
        boolean lastY = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y && !lastY) {
                fieldCentric = !fieldCentric;
            }
            lastY = gamepad1.y;

            if (gamepad1.back) {
                Pose2d p = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(p.getX(), p.getY(), 0));
            }

            double scale = gamepad1.left_bumper ? SLOW_MODE_SCALE : 1.0;
            // Stick forward is negative, so invert Y. Road Runner: +x forward, +y left, +heading CCW.
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).times(scale);
            if (fieldCentric) {
                input = input.rotated(-drive.getPoseEstimate().getHeading());
            }
            drive.setWeightedDrivePower(new Pose2d(input, -gamepad1.right_stick_x * scale));
            drive.update();

            mech.update(gamepad1.right_trigger - gamepad1.left_trigger, gamepad1.right_bumper);

            Pose2d pose = drive.getPoseEstimate();
            telemetry.addData("Drive mode", fieldCentric ? "FIELD-centric" : "robot-centric");
            telemetry.addData("Launcher", "%.0f tps %s", mech.getLauncherVelocity(),
                    mech.isLauncherReady() ? "(READY)" : "");
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.0f°", pose.getX(), pose.getY(),
                    Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }

        mech.stop();
    }
}
