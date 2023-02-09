using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto
{
    [Document(IndexName = "netAppStatus-idx", StorageType = StorageType.Json, Prefixes = new[] { "NetAppStatus" })]
    public class NetAppStatusDto : Dto
    {
        [Indexed]
        [RedisIdField]
        public override string Id { get; set; }

        [Indexed]
        public string Name { get; set; }

        [Indexed]
        public int HardLimit { get; set; }

        [Indexed]
        public int OptimalLimit { get; set; }

        [Indexed]
        public int? CurrentRobotsCount { get; set; }

        [Indexed]
        public DateTimeOffset Timestamp { get; set; }

        public override BaseModel ToModel()
        {
            var dto = this;
            return new NetAppStatusModel()
            {
                Id = Guid.Parse(dto.Id!),
                Name = dto.Name,
                HardLimit = dto.HardLimit,
                OptimalLimit = dto.OptimalLimit,
                CurrentRobotsCount = dto.CurrentRobotsCount,
                Timestamp = dto.Timestamp
            };
        }

    }
}
