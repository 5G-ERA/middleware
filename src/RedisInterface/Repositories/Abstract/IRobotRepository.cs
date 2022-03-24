﻿using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories.Abstract
{
    public interface IRobotRepository : IBaseRepository<RobotModel>
    {
        Task<List<RelationModel>> GetRelation();
    }
}
