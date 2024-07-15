package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 61.5, Math.toRadians(90)))

                                // FAR BLUE SHARED
                                //Preload Case A
                                /*.lineToSplineHeading(new Pose2d(-36,37, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-36,11, Math.toRadians(180)))*/

                                //Preload Case C
                                /*.setTangent(Math.toRadians(170))
                                .splineToLinearHeading(new Pose2d(-56, 16, Math.toRadians(220)), Math.toRadians(270))

                                .lineToSplineHeading(new Pose2d(-40, 11, Math.toRadians(180))) // score */
                                //Preload CaseB
                                /*.lineToSplineHeading(new Pose2d(-48,23, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-48,11, Math.toRadians(180)))

                                //Score
                                .lineToSplineHeading(new Pose2d(20, 11, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(48, 44, Math.toRadians(180)), Math.toRadians(30))

                                //Intake
                                .setTangent(Math.toRadians(205))
                                .splineToSplineHeading(new Pose2d(20, 11, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-20, 11, Math.toRadians(180)), Math.toRadians(180))

                                //Score
                                .lineToSplineHeading(new Pose2d(48, 9, Math.toRadians(180)))*/


                                // FAR BLUE
                                //Preload Case A and C
                                /*.lineToSplineHeading(new Pose2d(-36,37, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-36,11, Math.toRadians(180)))

                                //Preload CaseB
                                .lineToSplineHeading(new Pose2d(-48,23, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-48,11, Math.toRadians(180)))

                                //Score
                                .lineToSplineHeading(new Pose2d(20, 11, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(48, 48, Math.toRadians(180)), Math.toRadians(30))

                                //Intake
                                .setTangent(Math.toRadians(235))
                                .splineToLinearHeading(new Pose2d(20, 11, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-20, 11, Math.toRadians(180)), Math.toRadians(180))

                                //Score
                                .lineToSplineHeading(new Pose2d(28, 11, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(48, 24, Math.toRadians(190)), Math.toRadians(30))
*/
                                /* FAR RED SHARED
                                 //Preload Case A
                                 .setTangent(Math.toRadians(170))
                                 .splineToLinearHeading(new Pose2d(-54, -16, Math.toRadians(130)), Math.toRadians(90))

                                 .lineToSplineHeading(new Pose2d(-40, -11, Math.toRadians(180))) // score

                                 //Preload Case C
                                 //.lineToSplineHeading(new Pose2d(-36,-37, Math.toRadians(180)))
                                 //.lineToSplineHeading(new Pose2d(-36,-11, Math.toRadians(180)))

                                 //Preload CaseB
                                // .lineToSplineHeading(new Pose2d(-48,-25, Math.toRadians(180)))
                                // .lineToSplineHeading(new Pose2d(-48,-11, Math.toRadians(180)))

                                 //Score
                                 .setTangent(Math.toRadians(270))
                                 .lineToSplineHeading(new Pose2d(15, -11, Math.toRadians(180)))
                                 .splineToSplineHeading(new Pose2d(48, -33, Math.toRadians(180)), Math.toRadians(0))

                                 //Intake
                                 .setTangent(Math.toRadians(150))
                                 .splineToSplineHeading(new Pose2d(20, -11, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToSplineHeading(new Pose2d(-20, -11, Math.toRadians(180)), Math.toRadians(180))

                                 //Score
                                 .lineToSplineHeading(new Pose2d(48, -9, Math.toRadians(180)))*/


                                /*FAR RED
                                 //Preload Case A
                                .setTangent(Math.toRadians(170))
                                .splineToLinearHeading(new Pose2d(-54, -16, Math.toRadians(130)), Math.toRadians(90))

                                .lineToSplineHeading(new Pose2d(-40, -11, Math.toRadians(180))) // score

                                //Preload Case C
                                //.lineToSplineHeading(new Pose2d(-36,-37, Math.toRadians(180)))
                                //.lineToSplineHeading(new Pose2d(-36,-11, Math.toRadians(180)))

                                //Preload CaseB
                                .lineToSplineHeading(new Pose2d(-48,-25, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-48,-11, Math.toRadians(180)))

                                //Score
                               .setTangent(Math.toRadians(270))
                                .lineToSplineHeading(new Pose2d(20, -11, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(48, -33, Math.toRadians(180)), Math.toRadians(0))

                                //Intake
                                .setTangent(Math.toRadians(150))
                                .splineToSplineHeading(new Pose2d(20, -11, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-20, -11, Math.toRadians(180)), Math.toRadians(180))

                                //Score
                                .lineToSplineHeading(new Pose2d(28, -11, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(48, -24, Math.toRadians(170)), Math.toRadians(300))*/

                                // CLOSE BLUE PRELOAD
                                //Preload C
                                /* .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(0)), Math.toRadians(270))
                                 .setTangent(Math.toRadians(90))
                                 .splineToLinearHeading(new Pose2d(10,60,Math.toRadians(180)),Math.toRadians(90))
                                 .setTangent(Math.toRadians(0))
                                 .splineToSplineHeading(new Pose2d(45, 44, Math.toRadians(180)), Math.toRadians(0))*/

                                //Preload B
                                /* .setTangent(Math.toRadians(270))
                                 .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(270))
                                 .setTangent(Math.toRadians(90))
                                 .splineToLinearHeading(new Pose2d(10,60,Math.toRadians(180)),Math.toRadians(90))
                                 .setTangent(Math.toRadians(0))
                                 .splineToSplineHeading(new Pose2d(45, 44, Math.toRadians(180)), Math.toRadians(0))*/


                                // Preload A
                                /*.setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(23, 39, Math.toRadians(90)), Math.toRadians(270))
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(10,60,Math.toRadians(180)),Math.toRadians(90))
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45, 44, Math.toRadians(180)), Math.toRadians(0))*/

                                // CLOSE BLUE
                                //Preload C
                                //.lineToLinearHeading(new Pose2d(11.6,40,Math.toRadians(30)))
                                // .lineToSplineHeading(new Pose2d(45,28, Math.toRadians(180)))

                                //Preload B
                               /* .lineToSplineHeading(new Pose2d(11.6,36, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(45,35, Math.toRadians(180)))*/

                                // Preload A
                                /*.setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(23, 39, Math.toRadians(90)), Math.toRadians(270))
                                 .lineToSplineHeading(new Pose2d(45,42, Math.toRadians(180)))
*/

                                //Intake
                                /*.setTangent(Math.toRadians(145))
                                .splineToSplineHeading(new Pose2d(11.6, 59, Math.toRadians(180)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-36, 59, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-42, 55, Math.toRadians(218)), Math.toRadians(240))

                                //Score
                                .setTangent(Math.toRadians(10))
                                .splineToSplineHeading(new Pose2d(10, 60, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45, 44, Math.toRadians(180)), Math.toRadians(0))

                                //Intake 2
                                .setTangent(Math.toRadians(170))
                                .splineToSplineHeading(new Pose2d(11.6, 59, Math.toRadians(180)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-36, 59, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-42, 55, Math.toRadians(218)), Math.toRadians(240))

                               // Score 2
                                .setTangent(Math.toRadians(20))
                               .splineToSplineHeading(new Pose2d(10, 60, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45, 44, Math.toRadians(180)), Math.toRadians(0))*/

                                //CLOSE RED
                                //Preload A
                                /*.lineToSplineHeading(new Pose2d(11.6, -37, Math.toRadians(330)))
                                .lineToLinearHeading(new Pose2d(11.6,-48,Math.toRadians(270)))
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(46, -31, Math.toRadians(180)), Math.toRadians(0))*/


                                //Preload B
                                /*.lineToSplineHeading(new Pose2d(11.6,-36, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(45,-36, Math.toRadians(180)))
*/
                                //Preload C
                                //.setTangent(Math.toRadians(80))
                                //.splineToLinearHeading(new Pose2d(23, -44, Math.toRadians(270)), Math.toRadians(90))
                                //.lineToSplineHeading(new Pose2d(45,-42, Math.toRadians(180)))

                                //Intake
                                /*.setTangent(Math.toRadians(200))
                                .splineToSplineHeading(new Pose2d(11.6, -59, Math.toRadians(180)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-36, -59, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-42, -55, Math.toRadians(142)), Math.toRadians(120))

                                //Score
                                .setTangent(Math.toRadians(340))
                                .splineToSplineHeading(new Pose2d(10, -60, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45, -44, Math.toRadians(180)), Math.toRadians(0))

                                //Intake 2
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(11.6, -59, Math.toRadians(180)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-36, -59, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-42, -55, Math.toRadians(142)), Math.toRadians(120))

                                //Score 2
                                .setTangent(Math.toRadians(340))
                                .splineToSplineHeading(new Pose2d(10, -60, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45, -44, Math.toRadians(180)), Math.toRadians(0)) */

                                //CLOSE RED PRELOAD
                                //Preload A
                                //.lineToSplineHeading(new Pose2d(11,-30, Math.toRadians(180)))
                                //.lineToSplineHeading(new Pose2d(45,-26, Math.toRadians(180)))

                                //Park A
                                /*.setTangent(Math.toRadians(260))
                                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)), Math.toRadians(0))*/

                                //Park B
                                /*.setTangent(Math.toRadians(260))
                                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)), Math.toRadians(0))*/

                                //Preload C
                                /*.setTangent(Math.toRadians(80))
                                .splineToLinearHeading(new Pose2d(23, -40, Math.toRadians(270)), Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(23,-55, Math.toRadians(270)))
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(45, -44, Math.toRadians(180)), Math.toRadians(0))

                                //Park C
                                .setTangent(Math.toRadians(260))
                                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)), Math.toRadians(0))
*/
                                //Red Mid
                                //Preload A
                              /*  .lineToLinearHeading(new Pose2d(11.6, -39, Math.toRadians(315)))
                                .lineToLinearHeading(new Pose2d(50,-60,Math.toRadians(180)))
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(, -40.5, Math.toRadians(180)), Math.toRadians(0))*/

                                /*  //Preload B
                                 .lineToLinearHeading(new Pose2d(11.6, -33.5, Math.toRadians(270)))
                                  .lineToLinearHeading(new Pose2d(11.6,-60,Math.toRadians(180)))
                                  .setTangent(Math.toRadians(0))
                                  .splineToSplineHeading(new Pose2d(45, -35, Math.toRadians(180)), Math.toRadians(0))*/

                                //Preload C
                                /*  .lineToSplineHeading(new Pose2d(12.5, -37, Math.toRadians(210)))
                                  .setTangent(Math.toRadians(270))
                                  .splineToLinearHeading(new Pose2d(11.6,-60,Math.toRadians(180)),Math.toRadians(270))
                                  .setTangent(Math.toRadians(0))
                                  .splineToSplineHeading(new Pose2d(45, -42, Math.toRadians(180)), Math.toRadians(0))

                                  .setTangent(Math.toRadians(200))
                                  .splineToSplineHeading(new Pose2d(11.6, -59, Math.toRadians(180)), Math.toRadians(180))
                                  .lineToSplineHeading(new Pose2d(-36, -59, Math.toRadians(180)))
                                  .splineToSplineHeading(new Pose2d(-42, -55, Math.toRadians(142)), Math.toRadians(120))

                                  .setTangent(Math.toRadians(340))
                                  .splineToSplineHeading(new Pose2d(10, -60, Math.toRadians(180)), Math.toRadians(0))
                                  .splineToSplineHeading(new Pose2d(45, -44, Math.toRadians(180)), Math.toRadians(0))

                                  .setTangent(Math.toRadians(200))
                                  .splineToSplineHeading(new Pose2d(11.6, -59, Math.toRadians(180)), Math.toRadians(180))
                                  .lineToSplineHeading(new Pose2d(-36, -59, Math.toRadians(180)))
                                  .splineToSplineHeading(new Pose2d(-42, -55, Math.toRadians(142)), Math.toRadians(120))

                                  .setTangent(Math.toRadians(340))
                                  .splineToSplineHeading(new Pose2d(10, -60, Math.toRadians(180)), Math.toRadians(0))
                                  .splineToSplineHeading(new Pose2d(45, -44, Math.toRadians(180)), Math.toRadians(0))*/



                                //BLUE MID
                                //CASE A
                                /*.setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(23, 39, Math.toRadians(90)), Math.toRadians(270))
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(11.6,60,Math.toRadians(180)),Math.toRadians(90))
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45, 44, Math.toRadians(180)), Math.toRadians(0))*/

                                /*//CASE B
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(12, 39, Math.toRadians(90)), Math.toRadians(270))
                                .lineToLinearHeading(new Pose2d(45, 36, Math.toRadians(180)))
                                .lineTo(new Vector2d(-55, 36))
                                .lineTo(new Vector2d(45,36))
                                .lineTo(new Vector2d(-55, 36))
                                .lineTo(new Vector2d(45,36))
                                //
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-47,36,Math.toRadians(180)),Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-55,28,Math.toRadians(200)),Math.toRadians(230))
                                .setTangent(Math.toRadians(50))
                                .splineToSplineHeading(new Pose2d(-47,36,Math.toRadians(180)),Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45,36,Math.toRadians(180)),Math.toRadians(0))

                                .setTangent(Math.toRadians(90))
                                .forward(4)
                                .splineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)), Math.toRadians(0))*/

                                //CASE C
                               /* .lineToLinearHeading(new Pose2d(11.6,40,Math.toRadians(30)))
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(10,60,Math.toRadians(180)),Math.toRadians(90))
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45, 44, Math.toRadians(180)), Math.toRadians(0))*/

                                //.lineToLinearHeading(new Pose2d(11.6, -39, Math.toRadians(315)))

                /*.lineToLinearHeading(new Pose2d(50,-60,Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45, -40.5, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(325))
                .splineToSplineHeading(new Pose2d(28, -62.8, Math.toRadians(180)),  Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-11, -62.8, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-30, -47, Math.toRadians(52)), Math.toRadians(30))
                .setTangent(Math.toRadians(240))
                .splineToSplineHeading(new Pose2d(-11.6, -61.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(38, -61, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45, -34, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(325))
                .splineToSplineHeading(new Pose2d(25, -63, Math.toRadians(180)),  Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-12, -66, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-35, -48.5, Math.toRadians(52)), Math.toRadians(30))
                .setTangent(Math.toRadians(240))
                .splineToSplineHeading(new Pose2d(-13, -66, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(36, -59, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45, -30, Math.toRadians(180)), Math.toRadians(0))
                .forward(3.5)*/


                // R3D MID CASE A
              /*  .lineToSplineHeading(new Pose2d(13.5, -41, Math.toRadians(320)))

                .lineToLinearHeading(new Pose2d(25,-59,Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(45, -23, Math.toRadians(180)))


                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(new Pose2d(35, -56, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-36, -60, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-57.1, -44.6, Math.toRadians(162)), Math.toRadians(120))

                .setTangent(Math.toRadians(340))
                .lineToSplineHeading(new Pose2d(-12, -59.5, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(17, -57.5, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(45, -32, Math.toRadians(180)))

                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(35, -58, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-36, -62, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-57.5, -46, Math.toRadians(165)), Math.toRadians(120))

                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(-12, -62, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(17, -56.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -31, Math.toRadians(180)), Math.toRadians(0))
*/

                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}