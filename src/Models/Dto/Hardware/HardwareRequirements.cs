using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Hardware;

[Document(StorageType = StorageType.Json)]
public class HardwareRequirements
{
    [Indexed]
    public long? MinimumRam { get; set; }
    [Indexed]
    public int? MinimumNumCores { get; set; }

    public HardwareRequirements()
    {
    }

    public HardwareRequirements(int minimumRam, int minimumNumCores)
    {
        MinimumRam = minimumRam;
        MinimumNumCores = minimumNumCores;
    }
}