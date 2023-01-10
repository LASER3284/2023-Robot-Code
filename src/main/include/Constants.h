namespace constants {
    
    const static double Pi = 3.14159265358979323846;

    // Xbox Controller Button Assignments.
    enum XboxButtons 		{eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};

    // Xbox Controller Axis Assignments.
    enum XboxAxis			{eLeftAxisX = 0, eLeftAxisY, eLeftTrigger, eRightTrigger, eRightAxisX, eRightAxisY};

    // Logitech Flight Stick Button Assignments.
    enum LogButtons	 		{eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};

    template <class outputType>
    static outputType mapScalarToRange(double x, double out_min, double out_max, double in_min = -1, double in_max = 1) {
        return outputType( (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    }
}