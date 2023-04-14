using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Hardware;

[Document(StorageType = StorageType.Json)]
public class HardwareSpec
{
    [Indexed]
    public int? Cpu { get; set; }
    [Indexed]
    public long? Ram { get; set; }
    [Indexed]
    public long? StorageDisk { get; set; }
    [Indexed]
    public int? NumberCores { get; set; }
    [Indexed]
    public long? VirtualRam { get; set; }
}