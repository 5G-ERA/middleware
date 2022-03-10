-- This LUA script will query for all the policies inside redis with the attribute IsActive set to True. It will return to IsActive parameter and the Policy_Id.

redis.call('select', '6')

local KeysArray = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do
    table.insert(KeysArray, key);
end

return KeysArray
