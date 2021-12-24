namespace Domain.Core.Algoritms.Shared.Solvers;

public static class HookeJeevesSolve
{



    /// <summary>
    /// Finds minimum of specifief residual function by Hooke-Jeeves method (2D - x and y)
    /// </summary>
    /// <typeparam name="T">Can be TOABasePoint or TDOABaseLine</typeparam>
    /// <param name="eps">residual function</param>
    /// <param name="baseElements">a set of base elements of type T</param>
    /// <param name="xPrev">previous solution x coordinate</param>
    /// <param name="yPrev">previous solution y coordinate</param>
    /// <param name="z">z coordinate (depth), suppose to be a constant (known by direct measurement)</param>
    /// <param name="maxIterations">iterations limit</param>
    /// <param name="precisionThreshold">precision threshold (maximal step site to stop the algorithm)</param>
    /// <param name="simplexSize">initial size of the simplex</param>
    /// <param name="xBest">x coordinate of minimum</param>
    /// <param name="yBest">y coordinate of minimum</param>
    /// <param name="radialError">radial error, m</param>
    /// <param name="itCnt">number of iterations taken</param>
    public static void HJS2D_Solve<T>(Func<T[], double, double, double, double> eps,
                                      T[] baseElements, double xPrev, double yPrev, double z,
                                      int maxIterations, double precisionThreshold, double stepSize,
                                      out double xBest, out double yBest, out double radialError, out int itCnt)
    {
        #region Hooke-Jeeves 2D optimization

        double x0 = xPrev;
        double y0 = yPrev;
        double f0 = eps(baseElements, x0, y0, z);
        itCnt = 0;

        double dx = stepSize;
        double dy = stepSize;

        double x1 = x0, y1 = y0, f1 = f0, x2, y2, f2, f1t;

        while (((dx > precisionThreshold) || (dy > precisionThreshold)) && (itCnt < maxIterations))
        {
            #region explore around x1

            f1t = eps(baseElements, x1 + dx, y1, z);
            if (f1t > f0)
            {
                f1t = eps(baseElements, x1 - dx, y1, z);
                if (f1t > f0)
                {
                    dx /= 2.0;
                }
                else
                {
                    x1 -= dx;
                    f1 = f1t;
                }
            }
            else
            {
                x1 += dx;
                f1 = f1t;
            }

            f1t = eps(baseElements, x1, y1 + dy, z);
            if (f1t > f0)
            {
                f1t = eps(baseElements, x1, y1 - dy, z);
                if (f1t > f0)
                {
                    dy /= 2.0;
                }
                else
                {
                    y1 -= dy;
                    f1 = f1t;
                }
            }
            else
            {
                y1 += dy;
                f1 = f1t;
            }

            #endregion

            while ((f1 < f0) && (itCnt < maxIterations))
            {
                x2 = x1 * 2 - x0;
                y2 = y1 * 2 - y0;
                f2 = eps(baseElements, x2, y2, z);

                #region explore around x2

                f1t = eps(baseElements, x2 + stepSize, y2, z);
                if (f1t > f2)
                {
                    f1t = eps(baseElements, x2 - stepSize, y2, z);
                    if (f1t <= f2)
                    {
                        x2 -= stepSize;
                        f2 = f1t;
                    }
                }
                else
                {
                    x2 += stepSize;
                    f2 = f1t;
                }

                f1t = eps(baseElements, x2, y2 + stepSize, z);
                if (f1t > f2)
                {
                    f1t = eps(baseElements, x2, y2 - stepSize, z);
                    if (f1t <= f2)
                    {
                        y2 -= stepSize;
                        f2 = f1t;
                    }
                }
                else
                {
                    y2 += stepSize;
                    f2 = f1t;
                }

                #endregion

                x0 = x1;
                y0 = y1;
                f0 = f1;

                if (f2 <= f1)
                {
                    x1 = x2;
                    y1 = y2;
                    f1 = f2;
                    itCnt++;
                }
            }

            itCnt++;
        }

        #endregion

        xBest = x0;
        yBest = y0;
        radialError = Math.Sqrt(f0);
    }
}