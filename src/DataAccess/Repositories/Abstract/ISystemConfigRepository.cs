using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface ISystemConfigRepository
{
    /// <summary>
    ///     Gets system config instance
    /// </summary>
    /// <returns></returns>
    Task<SystemConfigModel?> GetConfigAsync();

    /// <summary>
    ///     Initializes system config values
    /// </summary>
    /// <param name="cfg"></param>
    /// <returns></returns>
    Task InitializeConfigAsync(SystemConfigModel cfg);
}