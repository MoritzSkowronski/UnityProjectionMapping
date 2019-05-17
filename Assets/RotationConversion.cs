using OpenCVForUnity.CoreModule;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Assets
{
    class RotationConversion
    {
        // Maths comes from Python's Relax http://svn.gna.org/svn/relax/1.3/maths_fns/rotation_matrix.py

        static float EULER_EPSILON = 0.00005f;

        static public Rotation RotationMatrixToEulerZXY (Mat R)
        {
            var i = 2;
            var j = 0; // EULER_NEXT[2]
            var k = 1; // EULER_NEXT[3]

            var cos_beta = Math.Sqrt (Math.Pow (R.get (i, i) [0], 2) + Math.Pow (R.get (j, i) [0], 2));

            double alpha, beta, gamma;
            if (cos_beta > EULER_EPSILON) {
                alpha = Math.Atan2 (R.get (k, j) [0], R.get (k, k) [0]);
                beta = Math.Atan2 (-R.get (k, i) [0], cos_beta);
                gamma = Math.Atan2 (R.get (j, i) [0], R.get (i, i) [0]);
            } else {
                alpha = Math.Atan2 (-R.get (j, k) [0], R.get (j, j) [0]);
                beta = Math.Atan2 (-R.get (k, i) [0], cos_beta);
                gamma = 0.0;
            }

            alpha = wrap_angles (alpha, 0.0, 2.0 * Math.PI); // Z
            beta = wrap_angles (beta, 0.0, 2.0 * Math.PI); // X
            gamma = wrap_angles (gamma, 0.0, 2.0 * Math.PI); // Y

            // errr... 180 - Z result seems right. Why?
            return new Rotation (RadianToDegree (beta), RadianToDegree (gamma), 180.0 - RadianToDegree (alpha));
        }

        static double wrap_angles (double angle, double lower, double upper)
        {
            var window = 2 * Math.PI;
            if (window - (upper - lower) > 1e-7)
                throw new ArgumentException ();

            while (true) {
                if (angle > upper)
                    angle = angle - window;
                else if (angle < lower)
                    angle = angle + window;
                else
                    break;
            }

            return angle;
        }

        private static double RadianToDegree (double angle)
        {
            return angle * (180.0 / Math.PI);
        }
    }
}