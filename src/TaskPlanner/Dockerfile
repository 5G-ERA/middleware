#See https://aka.ms/containerfastmode to understand how Visual Studio uses this Dockerfile to build your images for faster debugging.

FROM mcr.microsoft.com/dotnet/aspnet:6.0 AS base
WORKDIR /app
EXPOSE 80
EXPOSE 443

FROM mcr.microsoft.com/dotnet/sdk:6.0 AS build
WORKDIR /src
COPY ["TaskPlanner/TaskPlanner.csproj", "TaskPlanner/"]
RUN dotnet restore "TaskPlanner/TaskPlanner.csproj"
COPY . .
WORKDIR "/src/TaskPlanner"
RUN dotnet build "TaskPlanner.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish "TaskPlanner.csproj" -c Release -o /app/publish

FROM base AS final
ARG AWS_KEY_ID
ARG AWS_SECRET_KEY
WORKDIR /app
COPY --from=publish /app/publish .
ENV AWS_ACCESS_KEY_ID=$AWS_KEY_ID
ENV AWS_SECRET_ACCESS_KEY=$AWS_SECRET_KEY
ENTRYPOINT ["dotnet", "TaskPlanner.dll"]