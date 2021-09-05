package org.firstinspires.ftc.teamcode.drive.opmode.gradde.mathUtils;

public class GradientDescent {
    double learningRate;
    double tunerVar;
    /*
     * Gradient descent is the process of optimizing the next guess for kP
     * Iterative optimizing algorithm for finding a local minimum of a differential function
     */

    // 1) get Loss data from robot (AS)
    // 2) Get Cost = std Loss
    //          guess and error(in coding train video) is actually our Cost function because it tells us how off we are
    // 3) Update tuning constant kTunerVar += Cost(kTunerVar Current) * learning rate

    public GradientDescent(double startLearningRate , double startTunerVar){
        learningRate = startLearningRate;
        tunerVar = startTunerVar;
    }

    /**
     * Uses Gradient descent to get next tuner variable
     */

    public void getNextTunerVar(double[] loss, double learningRate){
        //data and cost done in AS later
        double costVal = getCost(loss);
        tunerVar += costVal * learningRate;
    }

    public double getCost(double[] loss){
        return StandardDev.getStandardDev(loss);
    }

    public double getTunerVar() {
        return tunerVar;
    }

    public void setLearningRate(double learningRate) {
        this.learningRate = learningRate;
    }

    public double getLearningRate() {
        return learningRate;
    }
}
