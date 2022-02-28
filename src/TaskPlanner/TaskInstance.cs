namespace Middleware.TaskPlanner
{
    public class TaskInstance
    {
        public Guid Id { get; set; } = Guid.NewGuid();
        public DateTime CreationDate { get; set; } = DateTime.Now;

    }
}
