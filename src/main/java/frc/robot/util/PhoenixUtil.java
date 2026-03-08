// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK())
                break;
        }
    }

    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    public static void registerSignals(BaseStatusSignal... signals) {
        // Okay I know this is kind of weird but I stole it from 6328 and what it does
        // is this:
        // - New array thats sized to current registered signals + new signals
        // - Copy old signals into new signals array
        // - Copy new signals into new signals array
        // - Redirect rioSignals pointer to new signals
        BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
        System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
        System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
        rioSignals = newSignals;
    }

    public static void refreshAll() {
        if(rioSignals.length > 0) {
            BaseStatusSignal.refreshAll(rioSignals);
        }
    }
}
