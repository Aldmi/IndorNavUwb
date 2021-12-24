using Domain.Core.Algoritms.TDoA.Model;

namespace Domain.Core.Algoritms.Shared.Solvers;

public static class HelderMeadSolve
{
    static readonly double NLM_A = 1.0;
    static readonly double NLM_B = 0.5;
    static readonly double NLM_R = 0.5;
    static readonly double NLM_Q = 0.5;
    static readonly double NLM_G = 2.0;

    public static readonly int NLM_DEF_IT_LIMIT = 600;
    public static readonly double NLM_DEF_PREC_THRLD = 1E-12;







    /// <summary>
    /// Finds minimum of specified residual function by Nelder-Mead (simplex) method (2D - x and y)
    /// </summary>
    /// <typeparam name="T">Can be TOABasePoint or TDOABaseLine</typeparam>
    /// <param name="eps">residual function</param>
    /// <param name="baseElements">a set of base elements of type T</param>
    /// <param name="xPrev">previous solution x coordinate</param>
    /// <param name="yPrev">previous solution y coordinate</param>
    /// <param name="z">z coordinate (depth), suppose to be a constant (known by direct measurement)</param>
    /// <param name="maxIterations">iterations limit</param>
    /// <param name="precisionThreshold">precision threshold</param>
    /// <param name="simplexSize">initial size of the simplex</param>
    /// <param name="xBest">x coordinate of minimum</param>
    /// <param name="yBest">y coordinate of minimum</param>
    /// <param name="radialError">radial error, m</param>
    /// <param name="itCnt">number of iterations taken</param>
    public static void NLM2D_Solve<T>(Func<T[], double, double, double, double> eps,
                                      T[] baseElements, double xPrev, double yPrev, double z,
                                      int maxIterations, double precisionThreshold, double simplexSize,
                                      out double xBest, out double yBest, out double radialError, out int itCnt)
    {
        #region Nelder-Mead 2D optimization

        bool isFinished = false;

        double tmp, tmp1;
        double[] xix = new double[3];
        double[] xiy = new double[3];
        double[] fxi = new double[3];
        double fl, fg, fh, fr, fe, fs;
        double xcx, xcy, xrx, xry, xex, xey, xsx, xsy;

        itCnt = 0;

        xix[0] = xPrev;
        xiy[0] = yPrev;
        xix[1] = xix[0] + simplexSize;
        xiy[1] = xiy[0] + simplexSize;
        xix[2] = xix[0] - simplexSize / 2;
        xiy[2] = xiy[0] + simplexSize / 2;

        while (!isFinished)
        {
            fxi[0] = eps(baseElements, xix[0], xiy[0], z);
            fxi[1] = eps(baseElements, xix[1], xiy[1], z);
            fxi[2] = eps(baseElements, xix[2], xiy[2], z);

            if (fxi[0] > fxi[1])
            {
                tmp = fxi[0]; fxi[0] = fxi[1]; fxi[1] = tmp;
                tmp = xix[0]; xix[0] = xix[1]; xix[1] = tmp;
                tmp = xiy[0]; xiy[0] = xiy[1]; xiy[1] = tmp;
            }

            if (fxi[0] > fxi[2])
            {
                tmp = fxi[0]; fxi[0] = fxi[2]; fxi[2] = tmp;
                tmp = xix[0]; xix[0] = xix[2]; xix[2] = tmp;
                tmp = xiy[0]; xiy[0] = xiy[2]; xiy[2] = tmp;
            }

            if (fxi[1] > fxi[2])
            {
                tmp = fxi[1]; fxi[1] = fxi[2]; fxi[2] = tmp;
                tmp = xix[1]; xix[1] = xix[2]; xix[2] = tmp;
                tmp = xiy[1]; xiy[1] = xiy[2]; xiy[2] = tmp;
            }

            fl = fxi[0];
            fg = fxi[1];
            fh = fxi[2];

            xcx = (xix[0] + xix[1]) / 2.0f;
            xcy = (xiy[0] + xiy[1]) / 2.0f;

            xrx = (1.0f + NLM_A) * xcx - NLM_A * xix[2];
            xry = (1.0f + NLM_A) * xcy - NLM_A * xiy[2];

            fr = eps(baseElements, xrx, xry, z);

            if (fr < fl)
            {
                xex = (1.0f - NLM_G) * xcx + NLM_G * xrx;
                xey = (1.0f - NLM_G) * xcy + NLM_G * xry;

                fe = eps(baseElements, xex, xey, z);

                if (fe < fr)
                {
                    xix[2] = xex;
                    xiy[2] = xey;
                }
                else
                {
                    xix[2] = xrx;
                    xiy[2] = xry;
                }
            }
            else
            {
                if ((fr > fl) && (fr < fg))
                {
                    xix[2] = xrx;
                    xiy[2] = xry;
                }
                else
                {
                    if ((fr > fg) && (fr < fh))
                    {
                        tmp = xix[2]; xix[2] = xrx; xrx = tmp;
                        tmp = xiy[2]; xiy[2] = xry; xry = tmp;
                        tmp = fxi[2]; fxi[2] = fr; fr = tmp;
                    }
                    else
                    {
                        if (fh < fr)
                        {
                            //
                        }
                    }

                    xsx = NLM_B * xix[2] + (1.0f - NLM_B) * xcx;
                    xsy = NLM_B * xiy[2] + (1.0f - NLM_B) * xcy;
                    fs = eps(baseElements, xsx, xsy, z);

                    if (fs < fh)
                    {
                        xix[2] = xsx;
                        xiy[2] = xsy;
                    }
                    else
                    {
                        xix[1] = (xix[1] - xix[0]) / 2.0f;
                        xiy[1] = (xiy[1] - xiy[0]) / 2.0f;
                        xix[2] = (xix[2] - xix[0]) / 2.0f;
                        xiy[2] = (xiy[2] - xiy[0]) / 2.0f;
                    }
                }
            }

            tmp = (fxi[0] + fxi[1] + fxi[2]) / 3.0f;
            tmp1 = ((fxi[0] - tmp) * (fxi[0] - tmp) +
                    (fxi[1] - tmp) * (fxi[1] - tmp) +
                    (fxi[2] - tmp) * (fxi[2] - tmp)) / 3.0f;

            isFinished = (++itCnt > maxIterations) || (Math.Sqrt(tmp1) <= precisionThreshold);
        }

        #endregion

        xBest = xix[0];
        yBest = xiy[0];
        radialError = Math.Sqrt(eps(baseElements, xBest, yBest, z));
    }
}