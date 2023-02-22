namespace Middleware.Models.Domain
{
    internal class SliceModel
    {
        public Guid Id { get; set; }
        public string Name { get; set; }
        public bool IsDynamic { get; set; }
        public string Type { get; set; }
    }
}