#See https://aka.ms/containerfastmode to understand how Visual Studio uses this Dockerfile to build your images for faster debugging.

FROM mcr.microsoft.com/dotnet/aspnet:6.0 AS base
WORKDIR /app
EXPOSE 80
EXPOSE 443

FROM mcr.microsoft.com/dotnet/sdk:6.0 AS build
WORKDIR /src
COPY ["ResourcePlanner/ResourcePlanner.csproj", "ResourcePlanner/"]
RUN dotnet restore "ResourcePlanner/ResourcePlanner.csproj"
COPY . .
WORKDIR "/src/ResourcePlanner"
RUN dotnet build "ResourcePlanner.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "ResourcePlanner.csproj" -c Release -o /app/publish

FROM base AS final
WORKDIR /app
COPY --from=publish /app/publish .
ENTRYPOINT ["dotnet", "ResourcePlanner.dll"]