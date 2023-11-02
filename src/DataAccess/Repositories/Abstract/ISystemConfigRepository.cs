using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface ISystemConfigRepository
{
    /// <summary>
    ///     Gets system config instance
    /// </summary>
    /// <returns></returns>
    Task<SystemConfigModel?> GetConfigAsync();
}