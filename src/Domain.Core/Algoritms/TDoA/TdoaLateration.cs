using Domain.Core.Algoritms.Shared;
using Domain.Core.Algoritms.Shared.Solvers;
using Domain.Core.Algoritms.TDoA.Model;
using Utils;

namespace Domain.Core.Algoritms.TDoA;

public class TdoaLateration
{
    public static readonly int MAX_ITER_DEF = 600;
    public static readonly double PREC_THR_DEF = 1E-12;
    public static readonly double Z_TAG = 1.5;
    public static readonly double SIMPLEX_SIZE_DEF = 10;


    ///  <summary>
    /// Оптимальная версия с заданными константами.
    /// Решает проблему навигации TDOA: ищет целевое местоположение по базовым точкам - точкам с известными местоположениями
    ///  и измеренное время прибытия.
    ///  Используется метод Нелдера-Мида (симплекс).
    ///  </summary>
    ///  <param name="basePoints">Массив базовых точек</param>
    ///  <param name="targetResult"></param>
    ///  <param name="xPrev">Предыдущая координата Х цели. NaN если не известно</param>
    ///  <param name="yPrev">Предыдущая координата Y цели. NaN если не известно</param>
    public static void TDOA_Locate2D_WithOptimalArgs(TOABasePoint[] basePoints, out TargetCalcResult targetResult, double xPrev = 0, double yPrev= 0)
        => TDOA_Locate2D(basePoints, xPrev, yPrev, Z_TAG, MAX_ITER_DEF, PREC_THR_DEF, SIMPLEX_SIZE_DEF, PhysicalConstants.SpeedOfLight, out targetResult);


    ///  <summary>
    /// Решает проблему навигации TDOA: ищет целевое местоположение по базовым точкам - точкам с известными местоположениями
    ///  и измеренное время прибытия.
    ///  Используется метод Нелдера-Мида (симплекс).
    ///  </summary>
    ///  <param name="basePoints">Массив базовых точек</param>
    ///  <param name="xPrev">Предыдущая координата Х цели. NaN если не известно</param>
    ///  <param name="yPrev">Предыдущая координата Y цели. NaN если не известно</param>
    ///  <param name="z_m">Высота цели (для 2D алгоритма константа) </param>
    ///  <param name="maxIterations">Макс число итераций для поиска координат (чтобы погрешщность была меньше чем precisionThreshold)</param>
    ///  <param name="precisionThreshold">максимально допустимая Погрешность</param>
    ///  <param name="simplexSize">размер симплекса</param>
    ///  <param name="velocity">скорость доставки сообщения от тэга до якоря (по воздуху это сокрость света)</param>
    ///  <param name="targetResult">Резутьтат латерации</param>
    public static void TDOA_Locate2D(TOABasePoint[] basePoints,
                                     double xPrev, double yPrev, double z_m,
                                     int maxIterations, double precisionThreshold, double simplexSize,
                                     double velocity, out TargetCalcResult targetResult)
    {
        var baseLines = TDOABaseline.BuildBaseLines(basePoints, velocity);


        TDOA_NLM2D_Navigation(baseLines, xPrev, yPrev, z_m,
                                    maxIterations, precisionThreshold, simplexSize,
                                    out var xBest, out var yBest, out var radialError, out var itCnt);


        targetResult = new TargetCalcResult(xBest, yBest, radialError, itCnt);

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
    private static void TDOA_NLM2D_Navigation(TDOABaseline[] baseLines,
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