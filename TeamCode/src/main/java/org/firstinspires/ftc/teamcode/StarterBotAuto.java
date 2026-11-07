package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Starter autonomous for the goBILDA StarterBot (team 5243), using Road Runner.
 *
 *   1. Drive from the start position to a launch position
 *   2. Spin up the launcher and fire the preloaded elements
 *   3. Drive to a park position
 *
 * !! The coordinates below are PLACEHOLDERS. Measure the real start/launch/park spots for this
 * season's field and replace them. Road Runner coordinates: inches, field center = (0, 0),
 * heading in radians. Try paths at https://learnroadrunner.com/ (MeepMeep) before running them.
 *
 * Do NOT run this until the drive is tuned (see the tuning order on learnroadrunner.com).
 */
@Config
@Autonomous(name = "5243 StarterBot Auto", group = "5243")
public class StarterBotAuto extends LinearOpMode {
    public static double START_X = -60, START_Y = 12, START_HEADING_DEG = 0;
    public static double LAUNCH_X = -24, LAUNCH_Y = 24, LAUNCH_HEADING_DEG = 45;
    public static double PARK_X = -36, PARK_Y = 48;
    public static double LAUNCH_TIME_S = 4.0; // how long to keep feeding the launcher

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        StarterBotMechanisms mech = new StarterBotMechanisms(hardwareMap);

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        drive.setPoseEstimate(startPose);

        TrajectorySequence toLaunch = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(LAUNCH_X, LAUNCH_Y, Math.toRadians(LAUNCH_HEADING_DEG)))
                .build();

        TrajectorySequence toPark = drive.trajectorySequenceBuilder(toLaunch.end())
                .lineTo(new Vector2d(PARK_X, PARK_Y))
                .build();

        telemetry.addData("Status", "Ready. Start pose: %s", startPose);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // 1. drive to launch position (launcher off while driving)
        drive.followTrajectorySequence(toLaunch);

        // 2. fire: update() spins up the launcher and only feeds once it's up to speed
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < LAUNCH_TIME_S) {
            mech.update(0, true);
            drive.update(); // keeps the heading/position held by the RR localizer
            telemetry.addData("Launcher", "%.0f tps", mech.getLauncherVelocity());
            telemetry.update();
        }
        mech.stop();

        // 3. park
        drive.followTrajectorySequence(toPark);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
