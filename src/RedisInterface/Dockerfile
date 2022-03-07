#See https://aka.ms/containerfastmode to understand how Visual Studio uses this Dockerfile to build your images for faster debugging.

FROM mcr.microsoft.com/dotnet/aspnet:6.0 AS base
WORKDIR /app
EXPOSE 80
EXPOSE 443

FROM mcr.microsoft.com/dotnet/sdk:6.0 AS build
WORKDIR /src
COPY ["src/RedisInterface/RedisInterface.csproj", "src/RedisInterface/"]
RUN dotnet restore "src/RedisInterface/RedisInterface.csproj"
COPY . .
WORKDIR "/src/src/RedisInterface"
RUN dotnet build "RedisInterface.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "RedisInterface.csproj" -c Release -o /app/publish

FROM base AS final
WORKDIR /app
COPY --from=publish /app/publish .
ENTRYPOINT ["dotnet", "RedisInterface.dll"]