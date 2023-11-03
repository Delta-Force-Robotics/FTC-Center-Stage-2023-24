package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideLevelThread extends Thread {
    public int currLevel = 1;
    private Telemetry telemetry;

    public void changeLevel(int currLevel){
        if (currLevel == 1) {
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - 🟩🟩🟩🟩");
        }
        else if (currLevel == 2) {
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - 🟩🟩🟩🟩\n" +
                            " 1 - ⬜️⬜️⬜️⬜️");
        }
        else if (currLevel == 3){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - 🟩🟩🟩🟩\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 4){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - 🟩🟩🟩🟩\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 5){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - 🟩🟩🟩🟩\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 6){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - 🟩🟩🟩🟩️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 7){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - 🟩🟩🟩🟩️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 8){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - 🟩🟩🟩🟩\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 9){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - 🟩🟩🟩🟩️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 10){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- 🟩🟩🟩🟩️️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
    }
}
