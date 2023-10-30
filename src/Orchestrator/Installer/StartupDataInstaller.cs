using Middleware.Common.Helpers;
using Middleware.Models.Domain;
using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Installer;

internal class StartupDataInstaller : IStartupDataInstaller
{
    /// <inheritdoc />
    public Task InitializeStartupDataAsync()
    {
        throw new NotImplementedException();
    }

    private PolicyModel CreateUrllcPolicy()
    {
        return new()
        {
            Id = Guid.Parse("70837862-0f57-403d-b1f3-5670c313b67a"),
            Name = "UrllcSliceLocation",
            Description = "Selects the location that has URLLC slices enabled",
            IsActive = true,
            IsExclusiveWithinType = 0,
            Priority = Priority.None,
            Scope = PolicyScope.Resource,
            Type = PolicyType.LocationSelection,
            Timestamp = DateTimeOffset.Now.DateTime
        };
    }

    private UserModel CreateDefaultUser()
    {
        var password = "middleware";
        var salt = AuthHelper.GetSalt();

        return new()
        {
            Id = Guid.NewGuid(),
            Name = "middleware",
            Password = AuthHelper.HashPasswordWithSalt(password, salt),
            Role = "admin",
            Salt = AuthHelper.StringifySalt(salt)
        };
    }
}