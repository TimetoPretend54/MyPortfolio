#!/usr/bin/env python
#
# TuringMachine.py
# Description: Program for Writing a Turing Machine Simulator
# Programmer: Adrian Beehner
# Date: 11/22/16

# Problem: Write a Turing Machine Simulator that uses 3 LCD screens to display the output,
#		The Turing Machine simulated should be somewhat complex (I choose multiplication)
#		Use all programs, drivers, etc so far and implement into the Turing Machine Simulator

# Imports
from __future__ import print_function
from LCDGroveDriver import *									# LCDGroveDriver.py MUST be in same directory as main file LCDTest2.py!
import time														# Required to import time module 
import SDL_Pi_TCA9545
from State import State

# Declare Variables
TCA9545_ADDRESS = (0x73)   										# 1110011 (A0+A1=VDD), the I2C Address/Bits
BUS = [0x01, 0x02, 0x04, 0x08]									# Set up each BUS address as an array ("list" is an array in python)
																# element 0 = BUS0, element 1 = BUS1, etc..., we are using BUS1, BUS2, BUS3 for 3 LCDs

# Set y=up Screen for each LCD (3 Screens)
for i in range(0, 3):											
	tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[i+1])
	setRGB(0,255,0)												# Set Backlight to Green
	setText("")													# Clear all 3 LCDS
	#time.sleep(1)
	i = i + 1
	
#  Set Title For Top Of Screen, Use setText to refresh the LCD Display (Clear the display) 
for i in range(0, 3):											
	tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[i+1])
	setRGB(0,255,0)												# Set Backlight to Green
	setText(" Turing M Sim: \n Adrian Beehner")				
	time.sleep(.05)
	i = i + 1
time.sleep(2)													# Clear
for i in range(0, 3):											
	tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[i+1])
	setRGB(0,255,0)												# Set Backlight to Green
	setText("")													# Clear all 3 LCDS
	time.sleep(.05)
	i = i + 1
	
#buf3=list("Grove -Update without erase, I like tacos in the evening and also I like the ninja dogs!")


#Move_L_R(*buf3)
#Move_R_L(*buf3)


# Class Turing Machine
class TuringMachine(object):
    #  The tape which will hold the data.
    tape = list()
    buf = list()
    #  Variables to hold the multiplications
    firstMult = 7
    secondMult = 2

    # 
    # Method to find the transition from a certain state based on the read char by the turing machine
    # parameter state object which represents the current state the machine is in. 
    # parameter readLetter char represents the current letter the turing machine is looking at.
    # return State object contains the new state the machine will traverse to, the direction it will take,
    # and the letter it will write before moving.
    #      
    @classmethod												# For @classmethod, whether we call the method from the instance or the class, it passes the class as first argument. Which is class Turing Machine here
    def getTransition(cls, state, readLetter):
        x = State()
        x.setName(state.name)
        if state.name == "q0":
            #  If read = # and also in q0, write # and move R.
            if readLetter == '#':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState(state.name)
                return x
                #  If read = 1 and in q0, replace the 1 with X, move pointer to next character on the right
                #  & update the state to q1
            elif readLetter == '1':
                x.setToWrite('X')
                x.setDirection(State.Directions.RIGHT)
                x.setNextState("q1")
                return x
        elif state.name == "q1":
            #  If the read letter = 1, write 1 (thus this does not change the 1), move the pointer to the right,
            #  and remain in same state.
            if readLetter == '1':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState(state.name)
                return x
            elif readLetter == '*':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState("q2")
                return x
        elif state.name == "q2":
            #  If the read letter is = 'Y' , keep it at the value y, move the pointer to the right, and remain in the same state.
            if readLetter == 'Y':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState(state.name)
                return x
            elif readLetter == '1':
                x.setToWrite('Y')
                x.setDirection(State.Directions.RIGHT)
                x.setNextState("q3")
                return x
            elif readLetter == '#':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.LEFT)
                x.setNextState("q6")
                return x
        elif state.name == "q3":
            #  If the read letter is = '1' , keep it at 1, move the pointer to right, and remain in the same state.
            if readLetter == '1':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState(state.name)
                return x
            elif readLetter == '#':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState("q4")
                return x
        elif state.name == "q4":
            #  If the read letter is = '0' , keep it at 0, move the pointer to the right, remain in the same state.
            if readLetter == '0':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState(state.name)
                return x
            elif readLetter == " ":
                x.setToWrite('0')
                x.setDirection(State.Directions.LEFT)
                x.setNextState("q5")
                return x
        elif state.name == "q5":
            #  If the read letter is = 0, or #, or 1, or Y , do not change it, and keep moving towarding the left
            #  Looking for the '*'.
            if readLetter == '0' or readLetter == '#' or readLetter == '1' or readLetter == 'Y':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.LEFT)
                x.setNextState(state.name)
                return x
            elif readLetter == '*':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState("q2")
                return x
        elif state.name == "q6":
            #  If the read letter is = 'Y' , change it to 1, move the pointer to the left to look for other Y's.
            #  Which will update them as 1. This is  happening after a whole multiplication iteration have occured
            #  and its then time to reset and do 1 more iteration.
            if readLetter == 'Y':
                x.setToWrite('1')
                x.setDirection(State.Directions.LEFT)
                x.setNextState(state.name)
                return x
            elif readLetter == '*':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.LEFT)
                x.setNextState("q7")
                return x
        elif state.name == ("q7"):
            if readLetter == '1':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.LEFT)
                x.setNextState(state.name)
                return x
            elif readLetter == 'X':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState("q8")
                return x
        elif state.name == "q8":
            if readLetter == '1':
                x.setToWrite(readLetter)
                x.setDirection(state.getDirection())
                x.setNextState("q0")
                print("\nfrom q8 returning state: " + x.getNextState())
                return x
            elif readLetter == '*':
                x.setToWrite(readLetter)
                x.setDirection(State.Directions.RIGHT)
                x.setNextState("q9")
                return x
        elif state.name == "q9":
			print("Accepted and result: " + str(cls.showResult()))
			for p in range (0, 3):
				tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[p+1])
				if (p == 0):
					setText_norefresh("Accepted!     ")
				if (p == 1):
					setText_norefresh("Stmt: " + str(cls.firstMult) + " * " + str(cls.secondMult))
				if (p == 2):
					setText_norefresh("Result: " + str(cls.showResult()) + "     ")
			n = 0															# to keep track of number of iteration
			if (len(cls.buf) < 17):
				tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[1])	# If length of 1 LCD
				setText_norefresh("\n" + ("".join(cls.buf)))
				time.sleep(.05)
			elif (len(cls.buf) < 33):
				for l in range (0, 2):
					tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[l+1])	# If length of 2 LCD						
					setText_norefresh("\n" + ("".join(cls.buf[16*(l)+n:16*(l+1)+n])))		    	
					time.sleep(.05)		
			elif (len(cls.buf) < 49):
				for p in range (0, 3):
					tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[p+1])	# If length of 3 LCDs				
					setText_norefresh("\n" + ("".join(cls.buf[16*(p)+n:16*(p+1)+n])))		    	
					time.sleep(.05)
			else:																							# Otherwise longer than 3 lcds
				for i in range(len(cls.buf)):										# -16, as our we want to move 
					for k in range (0, 3):	
						tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[k+1])								
						setText_norefresh("\n" + ("".join(cls.buf[16*(k)+n:16*(k+1)+n])))			    	
						time.sleep(.05)		
					if (16*3 + n) == len(cls.buf):									# 16*(# of Screens), we have 3 screens
						break						
					n = n + 1	
			sys.exit(0)
        return x

    # 
    #   Initialize the tape with the multiplicant and multiplier
    #      
    @classmethod														# For @classmethod, whether we call the method from the instance or the class, it passes the class as first argument.
    def initializeTape(cls, first_multiplicant, second_multiplicant):
        resultSize =  first_multiplicant * second_multiplicant
        cls.tape.append('#')
        #  Start of input (Put as many 1's here are the # for (# * multiplicant), so since its 7 * multiplicant at the momeent, we need to append seven 1's to the tape
        n = 0
        while n < first_multiplicant:
			cls.tape.append('1')
			n += 1
        cls.tape.append('*')											#  Multiplication Symbol

        i = 0
        while i < second_multiplicant:
            cls.tape.append('1')
            i += 1
        cls.tape.append('#')
        #  End of input
        #  Now Initialize free space for the answer in the tape.
        i = 0
        while i < resultSize:
            cls.tape.append(' ')
            i += 1

    @classmethod													# For @classmethod, whether we call the method from the instance or the class, it passes the class as first argument.
    def showResult(cls):
        result = 0
        i = 0
        while i < len(cls.tape):
            if cls.tape[i] == '0':
                result += 1
            i += 1
        return result

    @classmethod													# For @classmethod, whether we call the method from the instance or the class, it passes the class as first argument.
    def main(cls, args):
        #  The parameter is the two multipliers, can change to whatever multplier we want to test
        cls.initializeTape(cls.firstMult,cls.secondMult)										
        #  Initially the machine will assume that it is standing at state q0
        temp = State()
        temp.setName("q0")
        
        
        #  Process for printing the tape for preview in console
        i = 0
        while i < len(cls.tape):
            cls.buf = list()
            for p in range (0, 22):									# For setting read-write head into middle of 2nd LCD screen
				cls.buf.append(" ")
            m = 0
            for x in cls.tape:
				if (m == i):
					print ("["+ x + "]" "  ", end="")
					cls.buf.append("[")									# When adding to our tape list, the lcds need every element in list to be ONLY
					cls.buf.append(x)									# ONE CHARACTER LONG, otherwise, it will print out the list incorrectly,
					cls.buf.append("]")									# so everything must be appended to list by a a ONE character		
					cls.buf.append(" ")
				else:
					print(x + "  ", end="")
					cls.buf.append(x)
					cls.buf.append(" ")
				m = m + 1
            #print(cls.buf)
            
            n = 0													# to keep track of number of iterations									
            for k in range (0, 3):	
				tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[k+1])								
				setText_norefresh("\n" + ("".join(cls.buf[16*(k)+2*i:16*(k+1)+2*i])))		  
				time.sleep(0)										# NOTE, if you have really long tape and need head to stay on LCD (stay in place and not exit the screen), 
            n = n + 1												# use 16*(k)+2*i:16*(k+1)+2*i, instead of just +i, but also move i to the middle of the 2nd LCD screen, 
																	# so you can see the tape moving on both sides
            
            #  getTransition method returns a state object which contains info about the Direction the machine will take.
            print("\nTransition: " + temp.name + " at " + str(i))
            temp = cls.getTransition(temp, cls.tape[i])
            if temp.name == "":					 #  If no state was returned, then something wrong occured.
                print("\nRejected")			 #  This is a halt (might not even happen).
                sys.exit(0)
            print("\nState: " + temp.name + " Read Symbol:" + cls.tape[i] + "\n")
            # print("Writing: " + tape.get(i) + " , " + temp.getToWrite());
            print("What is the value: " + str(temp.getToWrite()))
            print("What is i: " + str(i))
            for p in range (0, 3):	
				tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[p+1])								
				if (p == 0):
					setText_norefresh("Transit at: " + str(i) + " ")
				if (p == 1):
					setText_norefresh("State: " + temp.name + "      ")
				if (p == 2):
					setText_norefresh("Read Symbol: " + cls.tape[i])
            cls.tape[i] = temp.getToWrite()
            
            # 
            #  The idea behind this procedure is that it for every letter in the tape
            #  it calls the getTransition method which return a state object containing information
            #  about the direction which the machine will take. If the machine is going right
            #  then the index of "i" is incremented by one, if the machine is going left, then the 
            #  index of "i" is decremented by one. Otherwise, the "i" remains the same.
            #  This procedure is repeated until the tape content has been iterated, paractically
            #  until the getTransition method returns the final answer.
            #              
            try:
                if temp.getDirection() == State.Directions.RIGHT:
                    i = i + 1
                    print("Moving Right")
                    print()
                    #for k in range (0, 3):	
					#tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[k+1])								
					#setText_norefresh("\n" + ("".join(buf[16*(k)+i:16*(k+1)+i])))		    	
					#time.sleep(0)	
                elif temp.getDirection() == State.Directions.LEFT:
                    i = i - 1
                    print("Moving Left")
                    print()
                    #for k in range (0, 3):	
					#tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = BUS[k+1])								
					#setText_norefresh("\n" + ("".join(buf[16*(k)+i:16*(k+1)+i])))		    	
					#time.sleep(0)	
                else:
                    print("Staying in place")
                    #  i = 0;
                    print()
            except Exception as e:
                i = 0
                print("Throwing an exception: " + str(e))
                print()
            next = State()
            next.setName(temp.getNextState())
            # print("next state: " + temp.getNextState());
            temp = next


if __name__ == '__main__':
    import sys
    TuringMachine.main(sys.argv)
