from durable.lang import *

with ruleset('day'):

	@when_all(c.first << (m.action == 'clean') & (m.object == 'lab'), 
	(m.action == 'fatigue') & (m.object == 'high') & (m.what_day == c.first.what_day))
	def MON(c):
		c.assert_fact({'what_day':c.first.what_day, 'action':'is', 'answer':'MON'})


	@when_all(c.first << (m.action == 'collect') & (m.object == 'driving data'),
	(m.action == 'fatigue') & (m.object == 'high') & (m.what_day == c.first.what_day))
	def TUE(c):
		c.assert_fact({'what_day':c.first.what_day, 'action':'is', 'answer':'TUE'})


	@when_all(c.first << (m.action == 'check') & (m.object == 'security'),
	(m.action == 'fatigue') & (m.object == 'low') & (m.what_day == c.first.what_day))
	def WED(c):
		c.assert_fact({'what_day':c.first.what_day, 'action':'is', 'answer':'WED'})


	@when_all(c.first << (m.action == 'leave work') & (m.object == 'early'),
	(m.action == 'fatigue') & (m.object == 'high') & (m.what_day == c.first.what_day))
	def THU(c):
		c.assert_fact({'what_day':c.first.what_day, 'action':'is', 'answer':'THU'})


	@when_all(c.first << (m.action == 'check') & (m.object == 'security'),
	(m.action == 'feel') & (m.object == 'happy') & (m.what_day == c.first.what_day))
	def FRI(c):
		c.assert_fact({'what_day':c.first.what_day, 'action':'is', 'answer':'FRI'})


	@when_all(c.first << (m.work == 'no'))
	def SAT_SUN(c):
		c.assert_fact({'what_day':c.first.what_day, 'action':'is', 'answer':'weekend'})






	@when_all((m.action == 'is') & (m.answer == 'MON'))
	def ML(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'class', 'object':'ML'})


	@when_all((m.action == 'is') & (m.answer == 'TUE'))
	def SAVE(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'save', 'object':'data'})


	@when_all((m.action == 'is') & (m.answer == 'WED'))
	def DL(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'class', 'object':'DL'})


	@when_all((m.action == 'is') & (m.answer == 'THU'))
	def AI_intro(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'class', 'object':'AI_intro'})


	@when_all((m.action == 'is') & (m.answer == 'FRI'))
	def Drink(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'drink', 'object':'beer'})





	@when_all(m.action == 'class')
	def drive(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'safe', 'object':'drive'})


	@when_all((m.action == 'save') & (m.object == 'data'))
	def preprocessing(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'preprocessing', 'object':'data'})


	@when_all((m.action == 'fatigue') & (m.object == 'high'))
	def coffee(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'bought', 'object':'coffee'})


	@when_all(m.work == 'no')
	def weekend(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':'take', 'object':'a rest'})


	@when_all(m.action == 'drink')
	def calm(c):
		c.assert_fact({'what_day':c.m.what_day, 'action':"don't be", 'object':'late'})



# print


	@when_all((+m.what_day) & (m.action == 'safe'))
	def output4(c):
		print("CAUTION: {0}, {2} {1}ly!".format(c.m.what_day, c.m.action, c.m.object))

	@when_all((+m.what_day) & (m.action == 'class'))
	def output(c):
		print("Fact1: {0}'s {1} is {2}".format(c.m.what_day, c.m.action, c.m.object))

	@when_all((+m.what_day) & (m.action == 'fatigue'))
	def output2(c):
		print("Fact2: {0}, my {1} is {2}".format(c.m.what_day, c.m.action, c.m.object))

	@when_all((+m.object) & ( m.action !='class') & (m.action != 'fatigue') & (m.action != 'safe'))
	def output3(c):
		print("Fact3: {0} {1} {2}".format(c.m.what_day, c.m.action, c.m.object))

	@when_all((+m.answer))
	def output4(c):
		print("Fact4: {0} {1} {2}\n\n".format(c.m.what_day, c.m.action, c.m.answer))






### TEST ###

#월요일
assert_fact('day', {'what_day' : 'day1', 'action' : 'clean', 'object' : 'lab', 'work':'yes'})
assert_fact('day', {'what_day' : 'day1', 'action' : 'fatigue', 'object' : 'high', 'work':'yes'})

#화요일
assert_fact('day', {'what_day' : 'day2', 'action' : 'collect', 'object' : 'driving data', 'work':'yes'})
assert_fact('day', {'what_day' : 'day2', 'action' : 'fatigue', 'object' : 'high', 'work':'yes'})

#수요일
assert_fact('day', {'what_day' : 'day3', 'action' : 'check', 'object' : 'security', 'work':'yes'})
assert_fact('day', {'what_day' : 'day3', 'action' : 'fatigue', 'object' : 'low', 'work':'yes'})


#목요일
assert_fact('day', {'what_day' : 'day4', 'action' : 'leave work', 'object' : 'early', 'work':'yes'})
assert_fact('day', {'what_day' : 'day4', 'action' : 'fatigue', 'object': 'high', 'work':'yes'})
#assert_fact('day', {'what_day' : 'one day', 'action' : 'check', 'object' : 'security'})

#금요일
assert_fact('day', {'what_day' : 'day5', 'action' : 'check', 'object' : 'security', 'work':'yes'})
assert_fact('day', {'what_day' : 'day5', 'action' : 'feel', 'object' : 'happy', 'work':'yes'})
#assert_fact('day', {'what_day' : 'another day', 'action' : 'leave work', 'object' : 'early'})


#WEEKEND
assert_fact('day', {'what_day' : 'day6', 'work':'no'})


