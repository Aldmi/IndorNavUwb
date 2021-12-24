namespace Domain.Core.Algoritms.TDoA.Model;

/// <summary>
/// Результат выполнения Латерации (определения координат цели)
/// </summary>
public class TargetCalcResult
{
    /// <summary>
    /// Координата X цели на плоскости
    /// </summary>
    public double Xtarget { get; }
    /// <summary>
    /// Координата Y цели на плоскости
    /// </summary>
    public double Ytarget { get; }
    /// <summary>
    /// Погрешность (Квадратнрый корень из последнего значения problemResidual функции)
    /// </summary>
    public double RadialError { get; }
    /// <summary>
    /// Кол-во итераций для достижения результата (при применении одного из методов решателя)
    /// </summary>
    public double ItCnt { get; }


    public TargetCalcResult(double xtarget, double ytarget, double radialError, double itCnt)
    {
        Xtarget = xtarget;
        Ytarget = ytarget;
        RadialError = radialError;
        ItCnt = itCnt;
    }


    public void Deconstruct(out double x, out double y)
    {
        x = Xtarget;
        y = Ytarget;
    }
}