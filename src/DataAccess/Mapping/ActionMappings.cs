﻿using Middleware.Common.Models;
using Middleware.DataAccess.Dto;
using Middleware.DataAccess.Dto.Hardware;

namespace Middleware.DataAccess.Mapping;

public static class ActionMappings
{
    public static ActionModel ToActionModel(this ActionDto dto)
    {
        return new ActionModel
        {
            Id = Guid.Parse(dto.Id!),
            Name = dto.Name,
            ActionPriority = dto.ActionPriority,
            MinimumNumCores = dto.HardwareRequirements!.MinimumNumCores,
            MinimumRam = dto.HardwareRequirements.MinimumRam,
            Tags = dto.Tags
        };
    }
    public static ActionDto ToActionDto(this ActionModel actionModel)
    {
        return new ActionDto
        {
            Id = actionModel.Id.ToString(),
            Name = actionModel.Name,
            ActionPriority = actionModel.ActionPriority,
            Tags = actionModel.Tags,
            HardwareRequirements = new HardwareRequirements(actionModel.MinimumRam, actionModel.MinimumNumCores)
        };
    }
}
