namespace Middleware.ResourcePlanner
{
    public class ResourcesInstance
    {
        public Guid Id { get; set; } = Guid.NewGuid();
        public DateTime CreationDate { get; set; } = DateTime.Now;

        public List<string> AvailableResources { get; } = new List<string>() { "CPU, GPU, RAM"};   
    }
}
