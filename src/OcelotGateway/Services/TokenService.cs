using System;
using Microsoft.IdentityModel.Tokens;
using Middleware.Common.Models;
using System.Security.Claims;
using System.Text;
using System.IdentityModel.Tokens.Jwt;

namespace Middleware.OcelotGateway.Services
{
    public class TokenService
    {
        public TokenModel GenerateToken(Guid id) 
        {
            var key = new SymmetricSecurityKey(Encoding.UTF8.GetBytes("my_secure_api_secret"));
            var credentials = new SigningCredentials(key, SecurityAlgorithms.HmacSha256Signature);
            var expirationDate = DateTime.UtcNow.AddHours(8);

            var claims = new[]
            {
                new Claim(ClaimTypes.Name, id.ToString()),
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
