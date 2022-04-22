using System.Text.Json.Serialization;
using AutoMapper;
using Middleware.Common.Models;
namespace Middleware.ResourcePlanner
{
    public interface IResourcePlanner
    {
        Task<TaskModel> Plan(TaskModel taskModel);
    }

    public class ResourcePlanner : IResourcePlanner
    {
        private object closestmachine;
        private Guid ActionSequence;
        RedisInterface.RedisApiClient _redisApiClient;

        public string ActionName { get; }
        public Guid ActionId { get; }
        public List<Priority> ActionPriority { get; }
        public string Placement { get; }

        private TaskModel _taskModel;
        private readonly IMapper _mapper;

        public ResourcePlanner(IHttpClientFactory factory, IMapper mapper)
        {
            HttpClient client = factory.CreateClient();
            _redisApiClient = new RedisInterface.RedisApiClient("http://redisinterface.api", client);
            _mapper = mapper;
        }

        public Guid GetTaskId()
        {
            if (_taskModel != null)
            {
                return _taskModel.Id;
            }
            return Guid.Empty;
        }

        public List<ActionModel> GetActionId(List<ActionModel> actionId) => actionId;

        public async Task<TaskModel> Plan(TaskModel taskModel)
        {
            _taskModel = taskModel;
            // actionPlanner will give resource planner the actionSequence. 

            // Get action sequence 
            List<ActionModel> actionSequence = taskModel.ActionSequence;
            if (actionSequence == null || actionSequence.Count == 0)
                throw new ArgumentException("Action sequence cannot be empty");


            // iterate throught actions in actionSequence
            foreach (ActionModel action in actionSequence)
            {
                List<RedisInterface.RelationModel> imagesTmp = (await _redisApiClient.ActionGetRelationByNameAsync(action.Id, "NEEDS")).ToList();
                List<RelationModel> images = new List<RelationModel>();
                foreach (RedisInterface.RelationModel imgTmp in imagesTmp)
                {
                    RelationModel relation = _mapper.Map<RelationModel>(imgTmp);
                    images.Add(relation);
                }

                // images -> list of all relations with images for the action
                foreach (RelationModel relation in images)
                {
                    RedisInterface.InstanceModel instanceTmp = await _redisApiClient.InstanceGetByIdAsync(relation.PointsTo.Id);
                    //map
                    InstanceModel instance = _mapper.Map<InstanceModel>(instanceTmp);

                    // add instance to actions 
                    action.Services.Add(instance);
                }

            }

            //List<ActionModel> sequence = new List<ActionModel>
            //{ new ActionModel() {
            //    ActionPriority = "3", Order = 2, Id = Guid.NewGuid() } };

            //List<ActionModel> sortedSequence = sequence.OrderBy(s => s.Order).ThenBy(s => int.Parse(s.ActionPriority)).ToList();


            //if (actionSequence.Count == 1)
            //    actionSequence = actionId;
            //else
            //    actionSequence = sortedSequence;


            // "ActionId": 2,
            //    "Order": 0,
            //    "ActionPriority": "1/2/3", 
            // We get the answer here. in Action Sequence 

            // for each action get required services (call redis API?)#
            //RedisInterface.ActionModel actionModel = new RedisInterface.ActionModel();
            //ActionModel actionModel1 = new ActionModel();


            // //get active policies: get closet machine.
            // List<PolicyModel> policy = new List<PolicyModel>();
            // policy.Add(new PolicyModel() { Description = (string)closestmachine } as PolicyModel);


            //query graph for resources to perform on task 
            //slam //object detection 
            //return back to action

            // List<ActionModel> policySequence = new List<ActionModel>();

            // TaskModel.ActionSequence actionModels = new TaskModel();


            // new List<ActionModel>(){ new ActionModel() { ImageName = "SLAM", Id = Guid.NewGuid() },
            // new ActionModel() { ImageName = "Object detection", Id = Guid.NewGuid() } };

            // RedisInterface.InstanceModel instancetmp = await _redisApiClient.InstanceGetByIdAsync(Guid.Empty); //TODO: chnage to real guid
            // InstanceModel instance = _mapper.Map<InstanceModel>(instancetmp);



            // //what actions needs to be instancaited to complete the action from RADU###############
            // //needs the redis query for the instances for the specified actions // give action id - and return the instnaces that are connected to the graph
            //RedisInterface.InstanceModel instanceModel = new RedisInterface.InstanceModel();
            // instanceModel.Id = Guid.NewGuid();



            //1.  get the images that are needed to be deployed 
            // get IMAGENAME SLAM and OBJECT DETECTION 
            //List<ImageName> images = new List<ImageName>();


            //2.  assigning these images to the actions 
            //ImageName imageName;


            ////3.  returning the task model with the updated images 
            //taskModel = new TaskModel();
            //new List<TaskModel>() { new TaskModel() { TaskModelActionSequence = "SLAM", Id = Guid.NewGuid() };
            //TaskModel taskModel1 = new TaskModel() {TaskModelActionSequence = "Object detection", Id = Guid.NewGuid() };
            // // query redis for best placement.
            // Get BEST Placement
            // if (EDGE_1 = closest machine and/or not occupied with any Task)

            // return "Edge_1 is free to perform task"
            //else if
            // (Cloud_1 = closest machine and/or not occupied with any Task)

            //return "Cloud_1 is free to perfrom task"
            //            // // the actors that are free to perform task 
            // // [edge1,cloud1]
            // // call redis API

            // //  


            // set services for action from result of the redis
            return taskModel;

        }





        //
    }

    public class Priority
    {
    }

    public class Order
    {
    }
}