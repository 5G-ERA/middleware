namespace Middleware.Models.Domain
{
    public class ActuatorModel
    {
        public string Name { get; set; }
        public string Type { get; set; }

        public int Number { get; set; }

        public List<string> Nodes { get; set; }
    }
}