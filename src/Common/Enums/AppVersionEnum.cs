using Middleware.Common.Attributes;

namespace Middleware.Common.Enums;

public enum AppVersionEnum
{
    [StringValue("Development")]
    Dev,
    [StringValue("Production")]
    Prod
}