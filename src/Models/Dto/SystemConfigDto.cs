using Middleware.Models.Domain;

namespace Middleware.Models.Dto;

public class SystemConfigDto : Dto
{
    /// <inheritdoc />
    public override string Id { get; set; } = default!;

    public string Ros2RelayContainer { get; init; } = "but5gera/ros2_relay_server:0.1.0";
    public string Ros1RelayContainer { get; init; } = "but5gera/relay_network_application:0.4.4";
    public string RosInterRelayNetAppContainer { get; init; } = "but5gera/inter_relay_network_application:0.4.4";

    /// <inheritdoc />
    public override BaseModel ToModel()
    {
        var d = this;
        return new SystemConfigModel
        {
            Id = Guid.Empty,
            Name = "System Config",
            Ros1RelayContainer = d.Ros1RelayContainer,
            Ros2RelayContainer = d.Ros2RelayContainer,
            RosInterRelayNetAppContainer = d.RosInterRelayNetAppContainer
        };
    }
}