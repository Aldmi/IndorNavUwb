using Domain.Core.Algoritms.Shared;
using Domain.Core.Algoritms.Shared.Solvers;
using Domain.Core.Algoritms.TDoA.Model;

namespace Domain.Core.Algoritms.TDoA;

public class TdoaLateration
{
    /// <summary>
    /// Solves TDOA navigation problem: searches for a target location by base points - points with known locations
    /// and measured times of arrival. Nelder-Mead (simplex) method is used.
    /// </summary>
    /// <param name="bases">A collection of base points</param>
    /// <param name="lat_prev_deg">Previous location latitude, signed degrees. Set it to NaN if previous location is unknown</param>
    /// <param name="lon_prev_deg">Previous location longitude, signed degrees. Set it to NaN if previous location is unknown</param>
    /// <param name="z_m">Target's depth, known by direct measurement</param>
    /// <param name="maxIterations">Nelder-Mead iterations limit</param>
    /// <param name="precisionThreshold">Precision threshold</param>
    /// <param name="simplexSize">Initial size of simplex, meters</param>
    /// <param name="el">Reference ellipsoid</param>
    /// <param name="velocity">Signal propagation velocity, e.g. speed of sound in m/s</param>
    /// <param name="lat_deg">Found latitude of the target</param>
    /// <param name="lon_deg">Found longitude of the target</param>
    /// <param name="radialError">Radial error. Square root from the final value of residual function</param>
    /// <param name="itCnt">Number of iterations taken</param>
    public static void TDOA_Locate2D_My(TOABasePoint[] basePoints,
                                     double xPrev, double yPrev, double z_m,
                                     int maxIterations, double precisionThreshold, double simplexSize,

                                     double velocity,
                                     out double xBest, out double yBest, out double radialError, out int itCnt)
    {
        var baseLines = TDOABaseline.BuildBaseLines(basePoints, velocity);

        TDOA_NLM2D_Navigation(baseLines, xPrev, yPrev, z_m,
                                    maxIterations, precisionThreshold, simplexSize,
                                    out xBest, out yBest, out radialError, out itCnt);

        //Algorithms.GeopointOffsetByDeltas(Algorithms.Deg2Rad(basesCentroid.Latitude), Algorithms.Deg2Rad(basesCentroid.Longitude),
        //                                  yBest, xBest,
        //                                  el,
        //                                  out yPrev, out xPrev);

    }


    /// <summary>
    /// Solves navigation problem: finds target location by TDOA (by base points with known location and times of arrival)
    /// with Nelder-Mead (simplex) algorithm. 2D problem (x and y are variables, z is supposed to be known by direct measurement)
    /// </summary>
    /// <param name="baseLines">A set of base lines represented each by two base points with known locations and time of arrival</param>
    /// <param name="xPrev">previous solution x coordinate</param>
    /// <param name="yPrev">previous solution x coordinate</param>
    /// <param name="z">z coordinate</param>
    /// <param name="maxIterations">iterations limit</param>
    /// <param name="precisionThreshold">precision threshold</param>
    /// <param name="simplexSize">initial size of simplex</param>
    /// <param name="xBest">x coordinate of target</param>
    /// <param name="yBest">y coordinate of target</param>
    /// <param name="radialError">radial error</param>
    /// <param name="itCnt">number of iterations taken</param>   
    public static void TDOA_NLM2D_Navigation(TDOABaseline[] baseLines,
        double xPrev, double yPrev, double z,
        int maxIterations, double precisionThreshold, double simplexSize,
        out double xBest, out double yBest, out double radialError, out int itCnt)
    {
        HelderMeadSolve.NLM2D_Solve<TDOABaseline>(TdoaProblemResidual.Eps_TDOA3D,
            baseLines, xPrev, yPrev, z,
            maxIterations, precisionThreshold, simplexSize,
            out xBest, out yBest, out radialError, out itCnt);
    }
}