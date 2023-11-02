namespace Middleware.Orchestrator.SliceManager;

internal interface ISliceManager
{
    /// <summary>
    ///     Configures a slice to handle the connection of the specific SIM Card
    /// </summary>
    /// <param name="robotId"></param>
    /// <param name="imsi"></param>
    /// <param name="sliceId"></param>
    /// <param name="dataRateUpLink"></param>
    /// <param name="dataRateDownLink"></param>
    /// <returns></returns>
    Task AttachImsiToSlice(Guid robotId, string imsi, string sliceId, int dataRateUpLink, int dataRateDownLink);
}