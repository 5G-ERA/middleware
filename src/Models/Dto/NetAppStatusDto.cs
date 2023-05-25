using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto
{
    [Document(IndexName = "netAppStatus-idx", StorageType = StorageType.Json, Prefixes = new[] { NetAppStatusDto.Prefix })]
    public class NetAppStatusDto : Dto
    {
        public const string Prefix = "NetAppStatus";

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

        [Indexed(Sortable = true)]
        public DateTimeOffset Timestamp { get; set; }

        public override BaseModel ToModel()
        {
            var dto = this;
            return new NetAppStatusModel()
            {
                Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
                Name = dto.Name,
                HardLimit = dto.HardLimit,
                OptimalLimit = dto.OptimalLimit,
                CurrentRobotsCount = dto.CurrentRobotsCount,
                Timestamp = dto.Timestamp.DateTime
            };
        }

    }
}
