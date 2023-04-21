local KeysArray = {};
local matches = redis.call('KEYS', '*')

for value,key in ipairs(matches) do
    table.insert(KeysArray, key);
end

return KeysArray