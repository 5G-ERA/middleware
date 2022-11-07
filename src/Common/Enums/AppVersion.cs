using Middleware.Common.Attributes;

namespace Middleware.Common.Enums;

public enum AppVersion
{
    [StringValue("Development")]
    Dev,
    [StringValue("Production")]
    Prod
}