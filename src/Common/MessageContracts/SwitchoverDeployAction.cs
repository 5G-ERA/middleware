namespace Middleware.Common.MessageContracts;

public record SwitchoverDeployAction : Message
{
    /// <summary>
    /// Location where to redeploy existing NetApp 
    /// </summary>
    public string Location { get; init; }
    /// <summary>
    /// Identifier of the deployed instance
    /// </summary>
    public Guid ActionId { get; init; }
    /// <summary>
    /// Identifier of the action plan
    /// </summary>
    public Guid ActionPlanId { get; init; }
}