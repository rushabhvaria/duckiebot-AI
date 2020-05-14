class StudentTests {
    static final double DEFAULT_TOL = 1e-4;

    static void assertArrayEquals(double[] expResult, double[] res, double tolerance, String testName) {
        String errorMsg = "";
        if (expResult == null || res == null) {
            errorMsg = "expResult or res is null!";
        }
        else if (expResult.length != res.length) {
            errorMsg = "array length mismatch: expResult.length: " + expResult.length + "  res.length: " + res.length;
        }
        for (int i = 0; i < res.length; i++) {
            double expected = expResult[i];
            double actual = res[i];
            if (errorMsg == "" && Math.abs(expected-actual) > tolerance) {
                errorMsg = "array element mismatch at index: " + i + " expResult: " + expected + "  actual: " + actual;
            }
        }
        if (errorMsg == "") {
            System.out.println("[Unit Test Passed!] " + testName);
        } else {
            System.out.println("[Unit Test Failed!] " + testName + "    " + errorMsg);
        }
    }

    static void testProportionalJointAngleControllerSingleStep() {
        double[] q = {0.1, 0.1, 0.1};
        double[] qd = {0.5, 0.5, 0.5};
        double Kp = 0.05;
        double[] res = SerialLink2.proportionalJointAngleController(q, qd, Kp);
        double[] expResult = {0.12, 0.12, 0.12};
        assertArrayEquals(expResult, res, DEFAULT_TOL, "testProportionalJointAngleControllerSingleStep");
    }

    static void testProportionalCartesianControllerSingleStep() {
        double xd = 0.0;
        double yd = 0.0;
        double theta_d = 0.0;
        Pose2 tool = new Pose2(6, 5.0, 0);
        double Kp = 0.05;
        double PI = (float)Math.PI;
        double[] q = {75.0/180.0*PI, -48.0/180.0*PI, -28.0/180.0*PI};

        double[] res = SerialLink2.proportionalCartesianController(xd, yd, theta_d, q, tool, Kp);
        double[] expResult = {1.4554016531328855, -1.1068564405384242, -0.3659985056000867};
        assertArrayEquals(expResult, res, DEFAULT_TOL, "testProportionalCartesianControllerSingleStep");
    }

    static void run() {
        testProportionalJointAngleControllerSingleStep();
        testProportionalCartesianControllerSingleStep();
    }
}
