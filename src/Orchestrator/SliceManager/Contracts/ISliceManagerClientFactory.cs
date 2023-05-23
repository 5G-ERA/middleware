namespace Middleware.Orchestrator.SliceManager.Contracts;

internal interface ISliceManagerClientFactory
{
    /// <summary>
    ///     Is Slicing mechanism available in current location
    /// </summary>
    /// <returns></returns>
    bool IsSlicingAvailable();

    /// <summary>
    ///     Creates Slice Manager client if available
    /// </summary>
    /// <returns></returns>
    ISliceManager CreateSliceManagerClient();
}