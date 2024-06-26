﻿using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto.Ros;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "robot-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class RobotDto : Dto
{
    public const string Prefix = "Robot";

    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;

    [Indexed]
    public string? Name { get; set; }

    [Indexed(JsonPath = "$.RosDistro")]
    public RosInfo Ros { get; init; } = new();

    [Indexed(Sortable = true)]
    public DateTimeOffset LastUpdatedTime { get; set; }

    [Indexed]
    public string? RobotModelName { get; set; }

    [Indexed]
    public string? RobotStatus { get; set; }

    [Indexed]
    public string? Manufacturer { get; set; }

    public Uri? ManufacturerUrl { get; set; }

    [Indexed]
    public long? MaximumPayload { get; set; }

    [Indexed]
    public long? MaximumTranslationalVelocity { get; set; }

    [Indexed]
    public string? CurrentTaskId { get; set; }

    [Indexed]
    public List<string>? TaskList { get; set; }

    [Indexed]
    public string? LocomotionTypes { get; set; }

    public long? MaximumRotationalVelocity { get; set; }

    public long? RobotWeight { get; set; }

    [Indexed]
    public int BatteryStatus { get; set; }

    public string? MacAddress { get; set; }

    public string? LocomotionSystem { get; set; }

    public string? LocomotionType { get; set; } // Compulsory field

    [Indexed]
    public List<Sensor>? Sensors { get; set; } = new();

    [Indexed]
    public List<Actuator>? Actuators { get; set; } = new();

    [Indexed]
    public List<Manipulator>? Manipulators { get; set; } = new();

    [Indexed]
    public List<DialogueModel>? Questions { get; init; } = new();

    [Indexed]
    public DateTimeOffset OnboardedTime { get; init; } = DateTimeOffset.Now;

    [Indexed]
    public List<string> QuestionIds { get; set; } = new();

    public HardwareSpec HardwareSpec { get; init; } = new();

    [Indexed]
    public string? SimCardNumber { get; init; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new RobotModel
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name!,
            BatteryStatus = dto.BatteryStatus,
            LocomotionSystem = dto.LocomotionSystem,
            Manufacturer = dto.Manufacturer,
            MacAddress = dto.MacAddress,
            ManufacturerUrl = dto.ManufacturerUrl,
            MaximumPayload = dto.MaximumPayload,
            RobotStatus = dto.RobotStatus,
            RobotWeight = dto.RobotWeight,
            LastUpdatedTime = dto.LastUpdatedTime.DateTime,
            RobotModelName = dto.RobotModelName,
            MaximumRotationalVelocity = dto.MaximumRotationalVelocity,
            MaximumTranslationalVelocity = dto.MaximumTranslationalVelocity,
            Cpu = dto.HardwareSpec.Cpu,
            Ram = dto.HardwareSpec.Ram,
            NumberCores = dto.HardwareSpec.NumberCores,
            StorageDisk = dto.HardwareSpec.StorageDisk,
            LocomotionType = dto.LocomotionType,
            RosDistro = dto.Ros.RosDistro,
            RosVersion = dto.Ros.RosVersion,
            ROSRepo = dto.Ros.RosRepo,
            Sensors = dto.Sensors?.Select(x => x.ToModel()).ToList(),
            Actuators = dto.Actuators?.Select(x => x.ToModel()).ToList(),
            Manipulators = dto.Manipulators?.Select(x => x.ToModel()).ToList(),
            OnboardedTime = dto.OnboardedTime.DateTime,
            Questions = dto.Questions,
            TaskList = dto.TaskList,
            CurrentTaskId = string.IsNullOrEmpty(dto.CurrentTaskId) ? Guid.Empty : Guid.Parse(dto.CurrentTaskId),
            ROSNodes = dto.Ros.RosNodes?.Select(x => x.ToModel()).ToList(),
            SimCardNumber = dto.SimCardNumber
        };
    }
}