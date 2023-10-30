namespace Middleware.Orchestrator.Installer;

internal interface IStartupDataInstaller
{
    /// <summary>
    ///     Initializes startup data for teh Middleware
    /// </summary>
    /// <returns></returns>
    Task InitializeStartupDataAsync();
}