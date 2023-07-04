namespace Middleware.Common.MessageContracts;

public record GatewayAddNetAppEntryMessage : Message
{
    public Guid ActionPlanId { get; set; }

    public Guid ServiceInstanceId { get; set; }

    public string NetAppName { get; set; }

    public string DeploymentLocation { get; init; }

    public string Route { get; init; }
}