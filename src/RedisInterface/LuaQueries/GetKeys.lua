-- This LUA script will query for all the policies inside redis with the attribute IsActive set to True. It will return to IsActive parameter and the Policy_Id.

--[Obsolete] 

redis.call('select', '3')

local KeysArray = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do
    table.insert(KeysArray, key);
end

return KeysArray
