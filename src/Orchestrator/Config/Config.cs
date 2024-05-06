using Middleware.Common.Config;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;

namespace Middleware.Orchestrator.Config;

/// <summary>
/// Fully optional configuration for the orchestrator deployment process
/// </summary>
public class Config
{
    public static Config NewConfig => new();
    private ILocation _location;
    private string _netAppDataKey;
    private MiddlewareConfig _middleware;
    private SystemConfigModel _system;
    private ICollection<string> _deploymentNames;

    private Config()
    {
    }

    /// <summary>
    ///  Gets the current system settings
    /// </summary>
    /// <exception cref="NullReferenceException">when value is null</exception>
    public SystemConfigModel System
    {
        get => _system ?? throw new NullReferenceException(nameof(System));
        set => _system = value;
    }

    /// <summary>
    /// Gets the location setting
    /// </summary>
    /// <exception cref="NullReferenceException">when value is null</exception>
    public ILocation Location
    {
        get => _location ?? throw new NullReferenceException(nameof(Location));
        set => _location = value;
    }

    /// <summary>
    /// Gets the data key to to persist NetApp data 
    /// </summary>
    /// <exception cref="NullReferenceException">when value is null</exception>
    public string NetAppDataKey
    {
        get => _netAppDataKey ?? throw new NullReferenceException(nameof(NetAppDataKey));
        set => _netAppDataKey = value;
    }

    /// <summary>
    /// Gets the local Middleware configuration
    /// </summary>
    /// <exception cref="NullReferenceException">when value is null</exception>
    public MiddlewareConfig Middleware
    {
        get => _middleware ?? throw new NullReferenceException(nameof(Middleware));
        set => _middleware = value;
    }

    /// <summary>
    /// Gets currently deployed netapp names
    /// </summary>
    /// <exception cref="NullReferenceException">when value is null</exception>
    public ICollection<string> DeploymentNames
    {
        get => _deploymentNames ?? throw new NullReferenceException(nameof(DeploymentNames));
        set => _deploymentNames = value;
    }
}

public static class ConfigExtensions
{
    public static Config WithLocation(this Config config, ILocation location)
    {
        config.Location = location;
        return config;
    }

    public static Config WithNetAppDataKey(this Config config, string netAppDataKey)
    {
        config.NetAppDataKey = netAppDataKey;
        return config;
    }

    public static Config WithSystem(this Config config, SystemConfigModel systemConfig)
    {
        config.System = systemConfig;
        return config;
    }

    public static Config WithMiddleware(this Config config, MiddlewareConfig middlewareConfig)
    {
        config.Middleware = middlewareConfig;
        return config;
    }

    public static Config WithDeploymentNames(this Config config, ICollection<string> deploymentNames)
    {
        config.DeploymentNames = deploymentNames;
        return config;
    }
}