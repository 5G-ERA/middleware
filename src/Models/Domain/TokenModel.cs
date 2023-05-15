namespace Middleware.Models.Domain
{
    public class TokenModel
    {
        public string Token { get; set; } = default!;
        public DateTime ExpirationDate { get; set; }
    }
}