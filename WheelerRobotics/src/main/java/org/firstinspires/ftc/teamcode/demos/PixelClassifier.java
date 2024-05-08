package org.firstinspires.ftc.teamcode.demos;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import java.util.HashMap;

@Config
public class PixelClassifier {
    public static int threshold = 300;

    HashMap<Integer, int[]> targets = new HashMap<>();
    // pixel: [r,g,b,a]
    public PixelClassifier() {

    }
    public void addClassifier(Integer number, int[] datapoint) {
        targets.put(number, datapoint);
    }
    public int classify(int[] detection) {
        int closest = -1;
        int minSum = threshold;
        for (int i : targets.keySet()) {
            int sum = 0;
            int[] target = targets.get(i);
            for (int j = 0; j < target.length; j++) {
                sum += abs(target[j] - detection[j]);
            }
            if (sum < minSum) {
                closest = i;
                minSum = sum;
            }
        }
        return closest;
    }
}
