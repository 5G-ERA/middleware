using System.Security.Cryptography;
using Microsoft.AspNetCore.Cryptography.KeyDerivation;

namespace Middleware.Common.Helpers;

public class AuthHelper
{
    public static byte[] GetSalt()
    {
        var salt = RandomNumberGenerator.GetBytes(128 / 8);
        return salt;
    }

    public static string StringifySalt(byte[] salt)
    {
        return Convert.ToBase64String(salt);
    }

    /// <summary>
    ///     Computes Hash + Salt for users'password
    /// </summary>
    /// <param name="password"></param>
    /// <param name="salt"></param>
    /// <returns> Hashed password </returns>
    public static string HashPasswordWithSalt(string password, byte[] salt)
    {
        var hashedPassword = Convert.ToBase64String(KeyDerivation.Pbkdf2(
            password,
            salt,
            KeyDerivationPrf.HMACSHA256,
            100000,
            256 / 8));
        return hashedPassword;
    }
}