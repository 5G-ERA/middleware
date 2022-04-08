using Middleware.Common.Attributes;

namespace Middleware.Common.Enums;

public enum ServiceStatusEnum
{
    /// <summary>
    /// Service is running and under workload
    /// </summary>
    [StringValue("Active")]
    Active,
    /// <summary>
    /// Service is inactive
    /// </summary>
    [StringValue("Down")]
    Down,
    /// <summary>
    /// Service in the process of instantiation
    /// </summary>
    [StringValue("Instantiating")]
    Instantiating,
    /// <summary>
    /// Service running without workload
    /// </summary>
    [StringValue("Idle")]
    Idle,
    /// <summary>
    /// Service in the process of instantiation
    /// </summary>
    [StringValue("Terminating")]
    Terminating,
    /// <summary>
    /// Service has encountered a problem
    /// </summary>
    [StringValue("Problem")]
    Problem
}