using Microsoft.IdentityModel.Tokens;
using System.Security.Claims;
using System.Text;
using System.IdentityModel.Tokens.Jwt;
using Middleware.Common.Config;
using Middleware.Models.Domain;
using Middleware.DataAccess.Repositories.Abstract;

namespace Middleware.OcelotGateway.Services
{
    public class TokenService
    {
        private JwtConfig _jwtConfig;
        

        public TokenService(JwtConfig jwtconfig)
        {
            
            _jwtConfig = jwtconfig ?? throw new ArgumentNullException(nameof(jwtconfig));
        }

        public TokenModel GenerateToken(Guid id, string userRole) 
        {
            var key = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(string.IsNullOrEmpty(_jwtConfig.Key) ? "noawsdefaultkeygenerator" : _jwtConfig.Key));
            var credentials = new SigningCredentials(key, SecurityAlgorithms.HmacSha256Signature);
            var expirationDate = DateTime.UtcNow.AddHours(4);
            
            var claims = new[]
            {
                new Claim(ClaimTypes.Name, id.ToString()),
                new Claim(ClaimTypes.Role, userRole),
                new Claim(JwtRegisteredClaimNames.Jti, Guid.NewGuid().ToString())
            };

            var token = new JwtSecurityToken(
                    audience: "redisinterfaceAudience",
                    issuer: "redisinterfaceIssuer",
                    claims: claims,
                    expires: expirationDate,
                    signingCredentials: credentials);
            var authToken = new TokenModel();
            authToken.Token = new JwtSecurityTokenHandler().WriteToken(token);
            authToken.ExpirationDate = expirationDate;

            return authToken;
        }
    }
}
