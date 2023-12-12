using Middleware.CentralApi.Services.Abstract;
using Middleware.Common.Job;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Quartz;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;

namespace Middleware.CentralApi.Jobs;
[DisallowConcurrentExecution]
public class UpdateOnlineLocationJob : BaseJob<UpdateOnlineStatusJob>
{
    private readonly ILogger _logger;
    private readonly IRobotRepository _robotRepository;
    public UpdateOnlineLocationJob(ILogger<UpdateOnlineStatusJob> logger, IRobotRepository robotRepository) : base(logger)
    {
        _logger = logger;
        _robotRepository = robotRepository;
    }

    protected override async Task ExecuteJobAsync(IJobExecutionContext context)
    {

        // get all CAN_REACH relations
        var relations = new List<RelationModel>();
        try
        {
            relations = await _robotRepository.GetRelationsWithName("CAN_REACH");
        } catch (Exception ex)
        {
            Logger.LogError(ex, "There was en error while getting relations: CAN_REACH;");
        }

        if (relations.Any())
            // chech all relations
            foreach (var rel in relations)
            {

                var relationAttributes = new List<KeyValuePair>();
                relationAttributes = rel.RelationAttributes;
                var robotId = rel.InitiatesFrom.Id;
                var locationId = rel.PointsTo.Id;
                if (relationAttributes != null)
                    foreach (KeyValuePair relAtribut in relationAttributes)
                    {
                        if (relAtribut.Key == "lastUpdatedTime")
                        {
                            string dateInput = (string)relAtribut.Value;
                            var parsedDateTime = DateTime.Parse(dateInput);

                            DateTime threeMinutesEarlier = DateTime.UtcNow.AddMinutes(-3);
                            // Check if last updated time was no later then 3 minutes ago from now.
                            if (parsedDateTime < threeMinutesEarlier)
                            {
                                RelationModel relationModel = new RelationModel();
                                var relationName = rel.RelationName;
                                relationModel.InitiatesFrom.Id = robotId;
                                relationModel.InitiatesFrom.Type = rel.InitiatesFrom.Type;
                                relationModel.RelationName = relationName;
                                relationModel.PointsTo.Id = locationId;
                                relationModel.PointsTo.Type = rel.PointsTo.Type;
                                try
                                {
                                    var isValid = await _robotRepository.DeleteRelationAsync(relationModel);
                                    if (!isValid)
                                    {
                                        Logger.LogError("Deleting relation did not succeed: formId: {robotId} relationName: {name} toId:{Id2}", robotId, relationName, locationId);
                                    }
                                }
                                catch (Exception ex)
                                {
                                    Logger.LogError(ex, "There was en error while deleteing relation formId: {robotId} relationName: {name} toId:{Id2}", robotId, relationName, locationId);
                                    //throw new ArgumentException("The relation was not deleted", nameof(model));
                                }
                            }
                        }
                    }
            }
    }
}
