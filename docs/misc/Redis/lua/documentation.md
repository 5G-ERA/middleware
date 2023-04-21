# LUA query system with Redis

## Launching from command line a lua script:

```
redis-cli -a password --eval GetEdges.lua , "[\"demo_edge\"]"
```

## Get specific value of key in redis-cli

```
JSON.GET c3d24fdb-76a9-46de-b4d6-66ec5f4bb5ab $.Name
```
