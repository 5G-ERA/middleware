using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment.RosCommunication;

namespace Middleware.Orchestrator.Deployment;

internal class RosConnectionBuilderFactory : IRosConnectionBuilderFactory
{
    private readonly ISystemConfigRepository _systemConfigRepository;

    public RosConnectionBuilderFactory(ISystemConfigRepository systemConfigRepository)
    {
        _systemConfigRepository = systemConfigRepository;
    }

    /// <inheritdoc />
    public async Task<IRosConnectionBuilder> CreateConnectionBuilder(RosDistro distro)
    {
        var cfg = await _systemConfigRepository.GetConfigAsync();

        IRosConnectionBuilder retVal = distro.RosVersion switch
        {
            RosVersion.Ros1 => new Ros1ConnectionBuilder(distro, cfg),
            RosVersion.Ros2 => new Ros2ConnectionBuilder(distro, cfg),
            _ => null
        };

        return retVal;
    }
}