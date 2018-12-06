/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
package com.artelys.knitro.examples.callbacks;

import com.artelys.knitro.api.KTRPutString;
import com.artelys.knitro.api.KTRISolver;

public class ExampleOutputRedirection extends KTRPutString
{
    @Override
    public int callbackFunction(String str, KTRISolver isolver) {
        System.err.print(str);
        return str.length();
    }
}
