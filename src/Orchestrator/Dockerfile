FROM mcr.microsoft.com/dotnet/aspnet:6.0 AS base
WORKDIR /app
EXPOSE 80
EXPOSE 443

FROM mcr.microsoft.com/dotnet/sdk:6.0 AS build
WORKDIR /src
COPY ["Orchestrator/Orchestrator.csproj", "Orchestrator/"]
RUN dotnet restore "Orchestrator/Orchestrator.csproj"
COPY . .
WORKDIR "/src/Orchestrator"
RUN dotnet build "Orchestrator.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "Orchestrator.csproj" -c Release -o /app/publish

FROM base AS final
ARG AWS_KEY_ID
ARG AWS_SECRET_KEY
WORKDIR /app
COPY --from=publish /app/publish .
ENV AWS_ACCESS_KEY_ID=$AWS_KEY_ID
ENV AWS_SECRET_ACCESS_KEY=$AWS_SECRET_KEY
ENTRYPOINT ["dotnet", "Orchestrator.dll"]
