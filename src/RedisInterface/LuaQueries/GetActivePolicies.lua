-- This LUA script will query for all the policies inside redis with the attribute IsActive set to True. It will return to IsActive parameter and the Policy_Id.

redis.call('select', '3')

local ActivePoliciesArray = {};
local matches = redis.call('KEYS', '*')
local index = 0

for value,key in ipairs(matches) do

    local PolicyId = redis.call('json.get', key, 'Id')
         local IsActive = redis.call('json.get', key, 'IsActive')
         local PolicyAllObject = redis.call('json.get',key)
         if (IsActive =="true") then table.insert(ActivePoliciesArray,PolicyAllObject) end

         index = index +1
end

return ActivePoliciesArray

