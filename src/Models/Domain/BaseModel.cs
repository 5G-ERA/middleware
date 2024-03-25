using Middleware.Models.ExtensionMethods;

namespace Middleware.Models.Domain;

public abstract class BaseModel
{
    public abstract Guid Id { get; set; }

    public abstract string Name { get; set; }

    public List<RelationModel>? Relations { get; set; } //= new List<RelationModel>();

    public abstract Dto.Dto ToDto();


    protected string GetNetAppAddress(string netAppName, Uri address)
    {
        var sanitized = netAppName.SanitizeToUriPath();
        string protocol = address.Scheme;
        string domain = address.Host;
        int port = address.Port;

        string modifiedAddress = $"{protocol}://{sanitized}.{domain}:{port}";

        return modifiedAddress;
    }

    protected string GetNetAppReportAddress(Uri address)
    {
        return $"{address}/status/netapp";
    }
}