namespace Middleware.Orchestrator.Installer;

internal interface IStartupDataInstaller
{
    /// <summary>
    ///     Initializes startup data for the Middleware
    /// </summary>
    /// <returns></returns>
    Task InitializeStartupDataAsync();
}