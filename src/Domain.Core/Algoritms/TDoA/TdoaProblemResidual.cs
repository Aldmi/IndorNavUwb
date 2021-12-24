using Domain.Core.Algoritms.Shared;
using Domain.Core.Algoritms.Shared.Solvers;
using Domain.Core.Algoritms.TDoA.Model;

namespace Domain.Core.Algoritms.TDoA;

public static class TdoaProblemResidual
{
    /// <summary>
    /// TDOA problem residual function
    /// </summary>
    /// <param name="baseLines">base lines, each represented by two base points with known locations and times of arrival</param>
    /// <param name="x">current x coordinate</param>
    /// <param name="y">current y coordinate</param>
    /// <param name="z">current z coordinate</param>
    /// <returns>value of residual function in specified location</returns>
    public static double Eps_TDOA3D(TDOABaseline[] baseLines, double x, double y, double z)
    {
        double result = 0;
        double eps;

        for (int i = 0; i < baseLines.Length; i++)
        {
            eps = Math.Sqrt((baseLines[i].X1 - x) * (baseLines[i].X1 - x) +
                            (baseLines[i].Y1 - y) * (baseLines[i].Y1 - y) +
                            (baseLines[i].Z1 - z) * (baseLines[i].Z1 - z)) -
                  Math.Sqrt((baseLines[i].X2 - x) * (baseLines[i].X2 - x) +
                            (baseLines[i].Y2 - y) * (baseLines[i].Y2 - y) +
                            (baseLines[i].Z2 - z) * (baseLines[i].Z2 - z)) - baseLines[i].PRD;
            result += eps * eps;
        }

        return result;
    }
}