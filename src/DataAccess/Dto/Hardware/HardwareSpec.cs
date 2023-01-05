using Redis.OM.Modeling;

namespace Middleware.DataAccess.Dto.Hardware;

[Document(StorageType = StorageType.Json)]
internal class HardwareSpec
{
    [Indexed]
    public long Cpu { get; set; }
    [Indexed]
    public long Ram { get; set; }
    [Indexed]
    public long StorageDisk { get; set; }
    [Indexed]
    public long NumberCores { get; set; }
}
