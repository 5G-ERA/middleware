using Middleware.Models.Domain;

namespace Middleware.Models.Dto;

public class SystemConfigDto : Dto
{
    /// <inheritdoc />
    public override string Id { get; set; } = default!;

    public string Ros2RelayContainer { get; init; } = default!;
    public string Ros1RelayContainer { get; init; } = default!;
    public string RosInterRelayNetAppContainer { get; init; } = default!;

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