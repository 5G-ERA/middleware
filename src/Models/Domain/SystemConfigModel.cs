using Middleware.Models.Dto;

namespace Middleware.Models.Domain;

public class SystemConfigModel : BaseModel
{
    /// <inheritdoc />
    public override Guid Id { get; set; } = Guid.Empty;

    /// <inheritdoc />
    public override string Name { get; set; } = "System Config";

    public string Ros2RelayContainer { get; set; } = default!;
    public string Ros1RelayContainer { get; set; } = default!;
    public string RosInterRelayNetAppContainer { get; set; } = default!;

    /// <inheritdoc />
    public override Dto.Dto ToDto()
    {
        var d = this;
        return new SystemConfigDto
        {
            Id = d.Id.ToString(),
            Ros1RelayContainer = d.Ros1RelayContainer,
            Ros2RelayContainer = d.Ros2RelayContainer,
            RosInterRelayNetAppContainer = d.RosInterRelayNetAppContainer
        };
    }
}