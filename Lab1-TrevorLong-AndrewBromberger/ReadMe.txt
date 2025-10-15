Lab1 Key Findings:

Preferred Sensor: Sonar
Reason: measurements appeared more reliable and less prone to calibraiton / weather changes (barometer was significantly impacted by low pressure and did not tare correctly during analysis)

------
Bang-bang /heuristic controller:

did not ever settle on an altitude and was unstable. Overshoot was significant (50%) and there was no real settling time.
It did react quickly to changes because reaction is based purely on if/then logic

------
PID controller

Once tuned provided more refined performance than the heuristic controller. Even when not perfectly tuned it often
provided stable responses that had real settling times. Overshoots tended to be smaller and control effort was more
varied due to the scaling on u from PID gains. Long term error was also definable vs. the bang-bang controller.

It did not react as quickly as the bang bang controller and did have more chattering, but as mentioned in other writeups
the chattering is not necessarily bad.

The most difficult part of PID controller is knowing where to start. A very untuned PID controller can be wildly unstable.

------
Comparison table

Metric                         If-Else Controller   PID Controller  Winner
---------------------------------------------------------------------------
Settling time [s]              inf                  23.90           PID
Steady-state error [m]         0.240                0.057           PID
Rise time [s]                  0.85                 3.76            Heuristic
Maximum overshoot [%]          53.9                 13.8            PID
Control chattering [switches/min] 44.1                 328.4           Heuristic
RMS error [m]                  0.320                0.180           PID

SUMMARY:
PID Controller wins: 4/6 metrics
If-Else Controller wins: 2/6 metrics
Ties: 0/6 metrics

Recommendation: PID controller