using Domain.Core.Algoritms.TDoA.Model;

namespace Domain.Core.Algoritms.TOA;

public static class ToaProblemResidual
{
    /// <summary>
    /// TOA problem residual function
    /// </summary>
    /// <param name="basePoints">base points with known locations and distances to them</param>
    /// <param name="x">current x coordinate</param>
    /// <param name="y">current y coordinate</param>
    /// <param name="z">current z coordinate</param>
    /// <returns>value of residual function in specified location</returns>
    public static double Eps_TOA3D(TOABasePoint[] basePoints, double x, double y, double z)
    {
        double result = 0;
        double eps = 0;

        for (int i = 0; i < basePoints.Length; i++)
        {
            eps = Math.Sqrt((basePoints[i].X - x) * (basePoints[i].X - x) +
                            (basePoints[i].Y - y) * (basePoints[i].Y - y) +
                            (basePoints[i].Z - z) * (basePoints[i].Z - z)) - basePoints[i].D;
            result += eps * eps;
        }

        return result;
    }

}