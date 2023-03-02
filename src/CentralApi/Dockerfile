FROM mcr.microsoft.com/dotnet/aspnet:6.0 AS base
WORKDIR /app
EXPOSE 80
EXPOSE 443

FROM mcr.microsoft.com/dotnet/sdk:6.0 AS build
WORKDIR /src
COPY ["src/CentralApi/CentralApi.csproj", "src/CentralApi/"]
RUN dotnet restore "src/CentralApi/CentralApi.csproj"
COPY . .
WORKDIR "/src/src/CentralApi"
RUN dotnet build "CentralApi.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "CentralApi.csproj" -c Release -o /app/publish /p:UseAppHost=false

FROM base AS final
WORKDIR /app
COPY --from=publish /app/publish .
ENTRYPOINT ["dotnet", "CentralApi.dll"]
