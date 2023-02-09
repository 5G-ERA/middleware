﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto
{
    [Document(IndexName = "robotStatus-idx", StorageType = StorageType.Json, Prefixes = new[] { "RobotStatus" })]
    public class RobotStatusDto : Dto
    {
        [Indexed]
        [RedisIdField]
        public override string Id { get; set; }

        [Indexed]
        public string Name { get; set; }

        [Indexed]
        public string ActionSequenceId { get; set; }

        [Indexed]
        public int? CurrentlyExecutedActionIndex { get; set; }

        [Indexed]
        public int BatteryLevel { get; set; }

        [Indexed]
        public DateTimeOffset Timestamp { get; set; }



        public override BaseModel ToModel()
        {
            var dto = this;
            return new RobotStatusModel()
            {
                Id = Guid.Parse(dto.Id!),
                Name = dto.Name,
                ActionSequenceId = Guid.Parse(dto.ActionSequenceId!),
                CurrentlyExecutedActionIndex = dto.CurrentlyExecutedActionIndex,
                BatteryLevel = dto.BatteryLevel,
                Timestamp = dto.Timestamp
            };
        }
    }
}
