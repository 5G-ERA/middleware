namespace Middleware.Models.Enums;

public enum ServiceStatus
{
    /// <summary>
    ///     Service is running and under workload
    /// </summary>
    Active,

    /// <summary>
    ///     Service is inactive
    /// </summary>
    Down,

    /// <summary>
    ///     Service in the process of instantiation
    /// </summary>
    Instantiating,

    /// <summary>
    ///     Service running without workload
    /// </summary>
    Idle,

    /// <summary>
    ///     Service in the process of instantiation
    /// </summary>
    Terminating,

    /// <summary>
    ///     Service has encountered a problem
    /// </summary>
    Problem
}