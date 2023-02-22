
-- This LUA script will query for all the registered tasks ids.

redis.call('select', '6')

local TaskIdArray = {};
local matches = redis.call('KEYS', '*')
local index = 0

for value,key in ipairs(matches) do

    local TaskId = redis.call('json.get', key, 'TaskId')
    table.insert(TaskIdArray,TaskId) end

return TaskIdArray
