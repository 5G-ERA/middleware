using Middleware.CentralApi.Services.Abstract;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.Common.Enums;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;

namespace Middleware.CentralApi.Services;

public class RobotService : IRobotService
{
    private readonly ILogger _logger;
    private readonly IRobotRepository _robotRepository;
    private readonly ILocationRepository _locationRepository;

    public RobotService(ILogger<RobotService> logger,
        IRobotRepository robotRepository,
        ILocationRepository locationRepository)
    {
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        _robotRepository = robotRepository ?? throw new ArgumentNullException(nameof(robotRepository));
        _locationRepository = locationRepository ?? throw new ArgumentNullException(nameof(locationRepository));
    }

    public async Task <List<string>>CreateRelation(RelationToLocationRequest data)
    {
        List<LocationNames> relGraph2s = data.Locations;
        // check if name of robot can be retrieved
        var robotName = await GetRobotNameByIdAsync(data.RobotId);
        if (robotName == null)
            throw new ArgumentNullException(nameof(robotName));
        

        List<string> errorsList = new List<string>();
        List<RelationModel> availableRrelations = new();
        var relationName = "CAN_REACH";

        availableRrelations = await _robotRepository.GetRelation(data.RobotId, relationName);

        foreach (LocationNames relGraph2 in relGraph2s)
        {
            if (relGraph2 != null)
            {
                var organisationName = relGraph2.OrganisationName;
                var locationName = relGraph2.LocationName;
                if (!string.IsNullOrEmpty(organisationName) && !string.IsNullOrEmpty(locationName))
                {
                    try
                    {
                        Location? location = await _locationRepository.GetSingleLocationByOrganizationAndNameAsync(organisationName, locationName);
                        if (location != null)
                        {
                            bool foundRelation = false;

                            RelationModel model = new();
                            model.RelationName = relationName;
                            model.InitiatesFrom.Id = data.RobotId;
                            // need revisel; InitiatesFrom.Type name is HARDCODED
                            model.InitiatesFrom.Type = "ROBOT";
                            model.InitiatesFrom.Name = robotName;
                            model.PointsTo.Id = location.Id;
                            // need revisel; model.PointsTo.Type name is HARDCODED
                            model.PointsTo.Type = "LOCATION";
                            model.PointsTo.Name = location.Name;
                            KeyValuePair lastUpdatedTime = new();
                            // need revisel;lastUpdatedTime.Key "Key name" is HARDCODED
                            lastUpdatedTime.Key = "lastUpdatedTime";
                            var lastUpdatedTimeValue = DateTime.UtcNow.ToString();
                            lastUpdatedTime.Value = lastUpdatedTimeValue;
                            var listRelationAttributes = new List<KeyValuePair>();
                            listRelationAttributes.Add(lastUpdatedTime);
                            model.RelationAttributes = listRelationAttributes;

                            foreach (RelationModel availableRelation in availableRrelations)
                            {
                                if (availableRelation.PointsTo.Id == location.Id)
                                {
                                    foundRelation = true;
                                    UpdateRelationAsync(model);
                                }
                            }
                            if (foundRelation == false)
                            {
                                AddRelationAsync(model);
                            }
                            
                        } else
                        {
                            _logger.LogWarning("Location {locationName} with organisation {organisationName} not found!", locationName, organisationName);
                        }
                    }
                    catch (Exception ex)
                    {
                        _logger.LogError(ex, "An error occurred: for Location {locationName} with organisation {organisationName} and robot id {data.robotId}; error: ", locationName, organisationName, data.RobotId);
                        var exceptionError = "An error occurred: for Location" +  locationName.ToString() + " with organisation " + organisationName.ToString() + " and robot id: " + data.RobotId.ToString() + " error: " + ex.ToString();

                        errorsList.Append(exceptionError);
                    }
                }
            }            
        }
        
        return errorsList;
    }

    public async Task<List<string>> DeleteRelation(RelationToLocationRequest data)
    {
        List<LocationNames> allLocations = data.Locations;

        // check if name of robot can be retreived
        var robotName = await GetRobotNameByIdAsync(data.RobotId);
        if (robotName == null)
            throw new ArgumentNullException(nameof(robotName));
        List<string> errorsList = new();

        var relationName = "CAN_REACH";

        foreach (LocationNames localLocation in allLocations)
        {
            var organisationName = localLocation.OrganisationName;
            var locationName = localLocation.LocationName;

            if (!string.IsNullOrEmpty(organisationName) && !string.IsNullOrEmpty(locationName))
            {
                try
                {
                    Location? location = await _locationRepository.GetSingleLocationByOrganizationAndNameAsync(organisationName, locationName);
                    if (location != null)
                    {
                        RelationModel model = new();
                        model.RelationName = relationName;
                        model.InitiatesFrom.Id = data.RobotId;
                        model.InitiatesFrom.Type = "ROBOT";
                        model.InitiatesFrom.Name = robotName;

                        model.PointsTo.Id = location.Id;
                        model.PointsTo.Type = "LOCATION";
                        model.PointsTo.Name = location.Name;

                        DeleteRelationAsync(model);
                    }
                    else
                    {
                        _logger.LogWarning("Location {locationName} with organisation {organisationName} not found!", locationName, organisationName);
                    }
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "An error occurred for deleteing Location {locationName} with organisation {organisationName} and robot id {data.robotId}; error: ", locationName, organisationName, data.RobotId);
                    var exceptionError = "An error occurred for deleteing Location" + locationName.ToString() + " with organisation " + organisationName.ToString() + " and robot id: " + data.RobotId.ToString() + " error: " + ex.ToString();

                    errorsList.Append(exceptionError);
                }
            }            
        }
        return errorsList;
    }

    //UpdateRelationAsync
    private async void AddRelationAsync(RelationModel model)
    {
        try
        {
            var isValid = await _robotRepository.AddRelationAsync(model);
            if (!isValid)
            {
                _logger.LogWarning("Adding relation did not succeed");
                //throw new ArgumentException("The relation was not created", nameof(model));
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
        }
    }
    private async void UpdateRelationAsync(RelationModel model)
    {
        try
        {
            var isValid = await _robotRepository.UpdateRelationAsync(model);
            if (!isValid)
            {
                _logger.LogWarning("Adding relation did not succeed");
                //throw new ArgumentException("The relation was not created", nameof(model));
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
        }
    }
    private async void DeleteRelationAsync(RelationModel model)
    {
        //RelationModel model2;
        try
        {
            var isValid = await _robotRepository.DeleteRelationAsync(model);
            if (!isValid)
            {
                _logger.LogWarning("Deleting relation did not succeed");      
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            //throw new ArgumentException("The relation was not deleted", nameof(model));
        }
    }

    private async Task<string?> GetRobotNameByIdAsync(Guid id)
    {
        var model = await _robotRepository.GetByIdAsync(id);
        if (model == null)
        {
            return null;
            //throw new ArgumentNullException("Robot was not found.", nameof(model));
        } else
        {
            var name = model.Name;
            return name;
        }        
    }
}