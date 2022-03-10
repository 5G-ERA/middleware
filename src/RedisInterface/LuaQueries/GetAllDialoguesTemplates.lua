-- Lua script to get all the questions and answers templates.

redis.call('select', '4')

local QuestionArray = {};
local matches = redis.call('KEYS', '*')
local index = 0

for value,key in ipairs(matches) do

         --local QuestionId = redis.call('json.get', key, 'Question_Id')
         --local Question = redis.call('json.get', key, 'Question')
         -- local IsSingleAnswer = redis.call('json.get',key, 'IsSingleAnswer')
         --local Answer = redis.call('json.get',key, 'Answer')
         local QuestionAllObject = redis.call('json.get',key)
         table.insert(QuestionArray,QuestionAllObject)


end

return QuestionArray
