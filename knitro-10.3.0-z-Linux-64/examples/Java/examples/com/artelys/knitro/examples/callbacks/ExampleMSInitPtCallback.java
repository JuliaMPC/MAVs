/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.callbacks;

import com.artelys.knitro.api.KTRMsInitptCallback;
import com.artelys.knitro.api.KTRISolver;

import java.util.List;
import java.util.Random;

public class ExampleMSInitPtCallback extends KTRMsInitptCallback
{
    private double _minimum;
    private double _maximum;
    private Random _random;
    private boolean _verbose;

    public ExampleMSInitPtCallback(double minimum, double maximum) {
        _minimum = minimum;
        _maximum = maximum;
        _random = new Random();
        _verbose = false;
    }

    @Override
    public int callbackFunction(
            int nSolveNumber,
            List<Double> xLoBnds,
            List<Double> xUpBnds,
            List<Double> x,
            List<Double> lambda,
            KTRISolver isolver) {

        if (_verbose)
        {
            System.out.format(System.getProperty("line.separator") + "using user-generated starting point in range [%1$s, %2$s]" + System.getProperty("line.separator"), _minimum, _maximum);
        }

        for (int i = 0; i < x.size(); i++) {
            x.set(i, _random.nextDouble() * (_maximum - _minimum) + _minimum);
        }

        return 0;
    }

    public void setVerbosity(boolean isVerbose) {
        _verbose = isVerbose;
    }
}
