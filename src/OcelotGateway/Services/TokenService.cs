using System;
using System.IdentityModel.Tokens.Jwt;
using System.Security.Claims;
using System.Text;
using Microsoft.IdentityModel.Tokens;
using Middleware.Common.Config;
using Middleware.Common.Models;

namespace Middleware.OcelotGateway.Services
{
    public class TokenService
    {
        private JwtConfig _jwtConfig;

        public TokenService(JwtConfig jwtConfig)
        {
            _jwtConfig = jwtConfig;
        }

        public TokenModel GenerateToken(Guid id)
        {
            var key = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(_jwtConfig.Key));
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
