package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleTankDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800);
        Trajectory traj2;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61.5, Math.toRadians(270)))

                                .lineToLinearHeading(new Pose2d(-45, -24.5, Math.toRadians(180)))

                                .lineTo(new Vector2d(-53,-11))
                                //.setTangent(Math.toRadians(160))
                                //.splineToLinearHeading(new Pose2d(-53,-11, Math.toRadians(180)), Math.toRadians(90))

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(49,-35, Math.toRadians(180)),Math.toRadians(280))

                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-55,-11,Math.toRadians(180)), Math.toRadians(180))

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(49,-29, Math.toRadians(180)), Math.toRadians(300))

                                .lineTo(new Vector2d(44,-29))
                                .setTangent(Math.toRadians(100))
                                .splineToLinearHeading(new Pose2d(60,-10, Math.toRadians(180)),Math.toRadians(0))

                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}