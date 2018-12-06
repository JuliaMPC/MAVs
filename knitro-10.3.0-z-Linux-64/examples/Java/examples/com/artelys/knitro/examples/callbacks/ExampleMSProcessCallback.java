/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

package com.artelys.knitro.examples.callbacks;

import com.artelys.knitro.api.KTRMSProcessCallback;
import com.artelys.knitro.api.KTRISolver;

import java.util.List;

public class ExampleMSProcessCallback extends KTRMSProcessCallback
{
    private int call = 0;
    private boolean enabled = true;

    @Override
    public int callbackFunction(
            List<Double> x,
            List<Double> lambda,
            double obj,
            List<Double> c,
            KTRISolver isolver) {
        if (!enabled) {
            return 0;
        }

        System.out.println("callbackMSProcess, call #"+(++call)+":");
        System.out.println("    Last solution: obj=" + obj);

        for (int i = 0; i < x.size(); i++) {
            System.out.format("\tx[%1$s]=%2$s", i, x.get(i));
        }
        System.out.println();

        return 0;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}